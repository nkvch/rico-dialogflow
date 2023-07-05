#!/usr/bin/env python
# encoding: utf8

import threading
import rospy
import actionlib
from std_msgs.msg import String, Bool
import tiago_msgs.msg
from rico_context.msg import HistoryEvent
import random

import time
import tempfile

from Levenshtein import distance

import copy
import sys
import os
from os import path
import codecs
import shutil
import unicodedata
import pl_nouns.odmiana as ro
from google.cloud import texttospeech as tts

import pyaudio
import subprocess
import json

import wave

# from dialogflow_agent.interfaces.DialogflowAgent import DialogflowAgent
from language_processor.srv import DetectIntentAndRetrieveParams, DetectIntentAndRetrieveParamsResponse
from task_database.srv import GetScenarioForIntent, GetParamsForScenario

def strip_inter(string):
    return string.replace(".", "").replace(",", "")


def remove_diacritics(string):
    # print string, type(string)
    u_text = unicodedata.normalize('NFKD', unicode(string, 'utf-8'))
    result = u_text.encode('ascii', 'ignore')

    return result

def remove_spaces(string):
    result = string.replace(' ', '-')

    return result

class SentencesContainer:
    def __init__(self, path):
        self.__sentences_map = {}
        self.__path = path
        self.__read()

    def __read(self):
        try:
            with open(self.__path + '/data.txt', 'r') as f:
                lines = f.readlines()
        except:
            lines = []

        for line in lines:
            fields = line.split('*')
            if len(fields) != 2:
                raise Exception(u'Wrong format of line "{}"'.format(line))
            ns = fields[0].decode('utf8')
            sound_fname = fields[1].decode('utf-8').strip()
            self.__sentences_map[ns] = sound_fname
            print ns, sound_fname

    def __normalizeSentence(self, sentence):
        return strip_inter(sentence).replace(u'\n', u' ').replace(u'\t', u' ').replace(u'*', u' ').strip().lower()

    def getSentence(self, sentence):
        assert isinstance(sentence, unicode)
        ns = self.__normalizeSentence(sentence)
        if not ns in self.__sentences_map:
            return None
        return self.__path + '/' + self.__sentences_map[ns]

    def addSentence(self, sentence, sound_fname):
        assert isinstance(sentence, unicode)
        ns = self.__normalizeSentence(sentence)
        if ns in self.__sentences_map:
            raise Exception(
                u'Sentence "{}" is already in the container'.format(ns))

        assert isinstance(ns, unicode)

        # Copy the sound file
        # Create unique name for the file
        while True:
            sound_fname_copy = u'sound_{}.wav'.format(
                random.randint(0, 1000000))
            if not path.exists(self.__path + '/' + sound_fname_copy):
                break
        shutil.copyfile(sound_fname, self.__path + '/' + sound_fname_copy)

        with open(self.__path + '/data.txt', 'a+') as f:
            # print ns
            # print  sound_fname_copy
            text = u'{}*{}'.format(ns, sound_fname_copy)
            # print text
            f.write(text.encode('utf8')+'\n')

        self.__sentences_map[ns] = sound_fname_copy


def play_sound(fname, start_pos):
    try:
        # open wave file
        wave_file = wave.open(fname, 'rb')

        # initialize audio
        py_audio = pyaudio.PyAudio()
        stream = py_audio.open(format=py_audio.get_format_from_width(wave_file.getsampwidth()),
                               channels=wave_file.getnchannels(),
                               rate=wave_file.getframerate(),
                               output=True)

        # skip unwanted frames
        n_frame_start = int(start_pos * wave_file.getframerate())
        wave_file.setpos(n_frame_start)

        # write desired frames to audio buffer
        # int(length * wave_file.getframerate())
        n_frames = wave_file.getnframes() - n_frame_start + 1
        frames = wave_file.readframes(n_frames)
        stream.write(frames)

        # close and terminate everything properly
        stream.close()
        py_audio.terminate()
        wave_file.close()
    except Exception as e:
        print e

# Action server for speaking text sentences


class SaySentenceActionServer(object):
    def __init__(self, name, playback_queue, odm, sentence_dict, sentences_container, cred_file):
        print 'init SaySentenceActionServer'
        self._action_name = name
        self._playback_queue = playback_queue
        self._odm = odm
        self._sentence_dict = sentence_dict
        self.__sentences_container = sentences_container
        self.__cred_file = cred_file
        self._as = actionlib.SimpleActionServer(
            self._action_name, tiago_msgs.msg.SaySentenceAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        # create messages that are used to publish feedback/result
        print 'SaySentenceActionServer callback runs'
        _feedback = tiago_msgs.msg.SaySentenceFeedback()
        _result = tiago_msgs.msg.SaySentenceResult()

        sentence_uni = goal.sentence.decode('utf-8')
        sentence_uni = self._odm.odmien(sentence_uni)

        prefix = u'niekorzystne warunki pogodowe'
        prefix_time_length = 2.1
        ss = strip_inter(sentence_uni).strip().upper()
        if sentence_uni.startswith(prefix):
            print u'detected "' + prefix + u'"'
            sentence_uni_no_prefix = sentence_uni[len(prefix):]
            pub_txt_msg.publish(sentence_uni_no_prefix)
            pub_context.publish(HistoryEvent('Rico', 'say', '"%s"' % sentence_uni_no_prefix, ''))
            sound_fname = self.__sentences_container.getSentence(
                sentence_uni_no_prefix)
            if sound_fname is None:
                print u'using dialogflow for sentence "{}"'.format(sentence_uni)
                sound_params = text_to_audio(sentence_uni, self.__cred_file)
                # Cut out the prefix
                sound_params = (
                    sound_params[0], sound_params[1], prefix_time_length)
                self.__sentences_container.addSentence(
                    sentence_uni_no_prefix, sound_params[0])
            else:
                print u'using cached sentence "{}"'.format(sentence_uni)
                # Cut out the prefix
                sound_params = (sound_fname, 'keep', prefix_time_length)
            best_d = 0
            best_k = None
        else:
            pub_txt_msg.publish(sentence_uni)
            best_k = ""
            best_d = 999
            print "Searching best match for", ss
            for k in self._sentence_dict.keys():
                # print k, v
                d = distance(k, ss)
            #    print k, d
                if d < best_d:
                    best_d = d
                    best_k = k
            sound_params = (self._sentence_dict[best_k], 'keep', 0.0)

        print u'Starting action for "' + sentence_uni + u'"'
        success = True
        if best_d < 5:
            print "Wiem co powiedzieć!", ss, best_k, sound_params
            sound_id = self._playback_queue.addSound(sound_params)

            while not self._playback_queue.finishedSoundId(sound_id) and not rospy.is_shutdown():
                # check that preempt has not been requested by the client
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    self._as.set_preempted()
                    success = False
                    break
                self._as.publish_feedback(_feedback)
                rospy.sleep(0.1)

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            _result.success = True
            self._as.set_succeeded(_result)

        print u'Ended action for "' + sentence_uni + u'"', success


# def get_audio_from_text_dialogflow(dialogflow_agent, text):
#     response = dialogflow_agent.repeat_text(text)

#     out = tempfile.NamedTemporaryFile(delete=False)
#     out.write(response.output_audio)
#     fname = out.name
#     out.close()
#     print('Audio content written to temporary file')

#     return response, (fname, 'delete', 0.0)

def text_to_audio(text, cred_file_incare_dialog):
    voice_name = 'en-US-Wavenet-A'
    language_code = "-".join(voice_name.split("-")[:2])
    text_input = tts.types.SynthesisInput(text=text)
    voice_params = tts.types.VoiceSelectionParams(
        language_code=language_code, name=voice_name
    )
    audio_config = tts.types.AudioConfig(audio_encoding=tts.enums.AudioEncoding.LINEAR16)

    client = tts.TextToSpeechClient.from_service_account_file(cred_file_incare_dialog)
    response = client.synthesize_speech(
        text_input, voice_params, audio_config
    )

    out = tempfile.NamedTemporaryFile(delete=False)
    out.write(response.audio_content)
    fname = out.name
    out.close()

    return (fname, 'delete', 0.0)

    # filename = "output.wav"
    # with open(filename, "wb") as out:
    #     out.write(response.audio_content)
    #     print "Generated speech saved to {}".format(filename)



def detect_intent_audio(dialogflow_agent, audio_file_path):
    with open(audio_file_path, 'rb') as audio_file:
        input_audio = audio_file.read()

    response = dialogflow_agent.detect_intent_audio(input_audio)

    out = tempfile.NamedTemporaryFile(delete=False)
    out.write(response.output_audio)
    fname = out.name
    out.close()

    return response, (fname, 'delete', 0.0)


def detect_intent_text(text):
    # response = dialogflow_agent.detect_intent_text(text)
    detect_intent_and_retrieve_params = rospy.ServiceProxy(
        'detect_intent_and_retrieve_params', DetectIntentAndRetrieveParams)
    
    response = detect_intent_and_retrieve_params(text)



    # out = tempfile.NamedTemporaryFile(delete=False)
    # out.write(response.output_audio)
    # fname = out.name
    # out.close()
    # print('Audio content written to temporary file')

    return response


pub_txt_msg = rospy.Publisher('txt_msg', String, queue_size=10)
pub_txt_voice_cmd_msg = rospy.Publisher(
    'txt_voice_cmd_msg', String, queue_size=10)
pub_cmd = rospy.Publisher('rico_cmd', tiago_msgs.msg.Command, queue_size=10)
pub_vad_enabled = rospy.Publisher('vad_enabled', Bool, queue_size=10)
pub_context = rospy.Publisher('/context/push', HistoryEvent, queue_size=10)

cred_file_incare_dialog = os.environ['GOOGLE_CONVERSATIONAL_DIALOGFLOW']


class PlaybackQueue:
    def __init__(self):
        self.__queue__ = []
        self.__queue_lock__ = threading.Lock()
        self.__sound_id__ = 0
        self.__current_sound_id__ = None

    def spin_once(self):
        fname = None
        self.__queue_lock__.acquire()
        if bool(self.__queue__):
            sound_id, (fname, keep_mode, start_time) = self.__queue__.pop(0)
            self.__current_sound_id__ = sound_id
        self.__queue_lock__.release()
        if fname:
            self.__playBlockingsound__(fname, start_time)
            self.__queue_lock__.acquire()
            self.__current_sound_id__ = None
            self.__queue_lock__.release()
            if keep_mode == 'delete':
                os.remove(fname)
            elif keep_mode == 'keep':
                # do nothing
                pass
            else:
                raise Exception('Wrong keep_mode: "' + keep_mode + '"')

    def finishedSoundId(self, sound_id):
        result = True
        self.__queue_lock__.acquire()
        if self.__current_sound_id__ == sound_id:
            result = False
        else:
            for s_id, _ in self.__queue__:
                if s_id == sound_id:
                    result = False
                    break
        self.__queue_lock__.release()
        return result

    def addSound(self, sound_file):
        assert sound_file[1] == 'keep' or sound_file[1] == 'delete'
        self.__queue_lock__.acquire()
        self.__queue__.append((self.__sound_id__, sound_file))
        result_sound_id = self.__sound_id__
        self.__sound_id__ = self.__sound_id__ + 1
        self.__queue_lock__.release()
        return result_sound_id

    def __playBlockingsound__(self, fname, start_time):
        pub_vad_enabled.publish(False)
        print 'playBlockingsound: BEGIN'
        print '  file:', fname

        play_sound(fname, start_time)
        print 'playBlockingsound: END'

        pub_vad_enabled.publish(True)

# def retireve_params(query, intent_name, params):
#     params_map = {}

#     for param in params:
#         if param.startswith('question_'):
#             params_map[param] = param.replace('question_', '')

#     params_for_nlp = []

#     for param in params:
#         if param in params_map:
#             params_for_nlp.append(params_map[param])
#         else:
#             params_for_nlp.append(param)

#     intent_name_for_nlp = intent_name.split(" AUTOGENERATED ")[0] if " AUTOGENERATED " in intent_name else intent_name
    
#     python3_path = '/home/nkvch/tiago_public_ws/src/rcprg/task_database/src/python3_script/venv/bin/python'
#     script_path = '/home/nkvch/tiago_public_ws/src/rcprg/task_database/src/python3_script/script.py'

#     data = subprocess.check_output([
#         python3_path,
#         script_path,
#         query,
#         intent_name_for_nlp
#     ] + params_for_nlp)

#     dict_data = json.loads(data)

#     print dict_data

#     for key in dict_data.keys():
#         if key in params_map.values():
#             new_key = params_map.keys()[params_map.values().index(key)]
#             dict_data[new_key] = dict_data.pop(key)
    
#     return dict_data


# def try_retrieve_parameters(query, intent_id, intent_name):
#     get_params_for_scenario = rospy.ServiceProxy('get_params_for_scenario', GetParamsForScenario)
#     get_scenario_for_intent = rospy.ServiceProxy('get_scenario_for_intent', GetScenarioForIntent)

#     scenario_id = get_scenario_for_intent(intent_id).scenario.id
#     params = get_params_for_scenario(int(scenario_id), True).params

#     print 'params', params

#     data = retireve_params(query, intent_name, params)

#     return data


# params_by_nlp = None
# is_prompting = False
# query_text = ''

def say(text, playback_queue):
    pub_txt_msg.publish(text)
    sound_file = text_to_audio(text, cred_file_incare_dialog)
    playback_queue.addSound(sound_file)
    pub_context.publish(HistoryEvent(
        'Rico',
        'say',
        '"%s"' % text,
        ''
    ))

def callback_common(response, playback_queue):
    if not response.matched:
        print 'No intent matched'
        say('Sorry i don\'t understand', playback_queue)
        return

    if not response.all_parameters_present:
        print 'Not all parameters present'
        say(response.fulfilling_question, playback_queue)
        return

    cmd = tiago_msgs.msg.Command()
    cmd.query_text = ''#query_text
    cmd.intent_name = unicode(response.intent_name, 'utf-8')
    for param_name, param in zip(response.retrieved_param_names, response.retrieved_param_values):

        # param_str = unicode(param)
        # colon_idx = param_str.find(':')
        # param_type = param_str[0:colon_idx]
        # # assert param_type == 'string_value'
        # param_value = param_str[colon_idx+1:].strip()[1:-1]
        # value_end = param_value.find('\"')
        # if value_end != -1:
        #     param_value = param_value[0:value_end]

        # print 'param_name: "' + param_name + '"'
        # print 'param_type: "' + param_type + '"'
        # print 'param_value: "' + param_value + '"'

        cmd.param_names.append(unicode(param_name, 'utf-8'))
        cmd.param_values.append(unicode(param, 'utf-8'))

    cmd.confidence = 1.0
    cmd.response_text = u'Okej'
    # print "CMD: ", cmd
    pub_cmd.publish(cmd)

    # this used to print Dialogflow responses to chat app

    # if len(response.query_result.fulfillment_text) > 0:
    #     pub_txt_msg.publish(response.query_result.fulfillment_text)
    # playback_queue.addSound(sound_file)


def callback(data, playback_queue):
    rospy.loginfo("I heard %s", data.data)
    pub_context.publish(HistoryEvent(
        'Rico',
        'hear',
        '"%s"' % data.data,
        ''
    ))
    response = detect_intent_text(data.data)

    callback_common(response, playback_queue)


def callback_wav(data, playback_queue):
    rospy.loginfo("I recorded %s", data.data)
    pub_context.publish(HistoryEvent(
        'Rico',
        'hear',
        '"%s"' % data.data,
        ''
    ))
    response = detect_intent_audio(data.data, cred_file_incare_dialog)
    pub_txt_voice_cmd_msg.publish(response.query_result.query_text)
    callback_common(response, playback_queue)


def callback_new_intent(data, dialogflow_agent):
    intent_name = data.trigger_phrase
    trigger_phrases = [data.trigger_phrase]
    question = data.questions[0]
    param_name = remove_spaces(remove_diacritics(question))
    parameters = [{'name': param_name, 'prompt': question + '?'}]
    autoresponses = ['dziękuję']
    dialogflow_agent.create_intent(intent_name, trigger_phrases, autoresponses, parameters)



class Odmieniacz:
    def __init__(self):
        self.o = ro.OdmianaRzeczownikow()

    def przypadki(self, word, przyp):
        print 'przypadki ', word, przyp
        blocks = self.o.getBlocks(word)
        print 'blocks', blocks
        if len(blocks) == 0:
            word_m = word
            lp = True
        else:
            m_lp = self.o.getMianownikLp(blocks)
            if len(m_lp) == 0:
                m_lm = self.o.getMianownikLm(blocks)
                word_m = m_lm[0]
                lp = False
            else:
                word_m = m_lp[0]
                lp = True

        if przyp == 'mianownik':
            word_p = word

        if przyp == 'biernik':
            if lp:
                word_p = self.o.getBiernikLp(blocks, mianownik=word_m)
                if len(word_p) == 0:
                    word_p = word_m
                else:
                    word_p = word_p[0]
            else:
                word_p = self.o.getBiernikLm(blocks, mianownik=word_m)
                if len(word_p) == 0:
                    word_p = word_m
                else:
                    word_p = word_p[0]

        if przyp == 'dopelniacz':
            if lp:
                word_p = self.o.getDopelniaczLp(blocks, mianownik=word_m)
                if len(word_p) == 0:
                    word_p = word_m
                else:
                    word_p = word_p[0]
            else:
                word_p = self.o.getDopelniaczLm(blocks, mianownik=word_m)
                if len(word_p) == 0:
                    word_p = word_m
                else:
                    word_p = word_p[0]

        if przyp == 'wolacz':
            if lp:
                word_p = self.o.getWolaczLp(blocks, mianownik=word_m)
                if len(word_p) == 0:
                    word_p = word_m
                else:
                    word_p = word_p[0]
            else:
                word_p = self.o.getWolaczLm(blocks, mianownik=word_m)
                if len(word_p) == 0:
                    word_p = word_m
                else:
                    word_p = word_p[0]

        return word_m, word_p

    def odmien(self, s):
        result = copy.copy(s)
        print "mam odmienic:", s
        while True:
            l_brace_idx = result.find('{')
            if l_brace_idx < 0:
                break
            r_brace_idx = result.find('}', l_brace_idx)
            if r_brace_idx < 0:
                break
            odm = result[l_brace_idx+1:r_brace_idx]
            print "odm:", odm
            quot_idx1 = odm.find('"')
            quot_idx2 = odm.find('"', quot_idx1+1)
            word_orig = odm[quot_idx1+1:quot_idx2]
            print word_orig
            sep_idx = odm.find(',', quot_idx2+1)
            przyp = odm[sep_idx+1:].strip()
            print "przyp:", przyp
            word_m, word_p = self.przypadki(word_orig, przyp)
            result = result[0:l_brace_idx] + word_p + result[r_brace_idx+1:]
        return result


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('talker', anonymous=True)
    rospy.wait_for_service('get_scenario_for_intent')
    rospy.wait_for_service('get_params_for_scenario')
    rospy.wait_for_service('detect_intent_and_retrieve_params')

    odm = Odmieniacz()
    playback_queue = PlaybackQueue()

    agent_name = rospy.get_param('~agent_name')
    data_dir = rospy.get_param('~data_dir')

    sentence_dict = {}
    from itertools import izip
    for sent, fname in izip(codecs.open(os.path.join(data_dir, "labels.txt"), encoding="utf-8"), open(os.path.join(data_dir, "files.txt"))):
        ss = unicode(sent)
        sentence_dict[strip_inter(ss).strip().upper()] = os.path.join(
            data_dir, fname.strip())
    '''
    rospy.sleep(1)
    key0 = sentence_dict.keys()[0]
    fname = sentence_dict[key0]
    print 'fname', fname
    play_sound(fname, 0.5)
    print 'end'
    #pygame.mixer.music.load(fname)
    #pygame.mixer.music.play(0, 0.5)
    rospy.sleep(2)
    exit(0)
    '''
    #text = raw_input('.')
    #response, sound_fname = detect_intent_text(agent_name, "me", u'niekorzystne warunki pogodowe ' + text, "pl")
    #sound_fname = (sound_fname[0], sound_fname[1], 0.6)
    #play_sound(sound_fname[0], 0.0)
    # print 'received response:', response.query_result
    # exit(0)

    if not 'GOOGLE_CONVERSATIONAL_DIALOGFLOW' in os.environ:
        raise Exception(
            'Env variable "GOOGLE_CONVERSATIONAL_DIALOGFLOW" is not set')
    # if not 'GOOGLE_CREDENTIALS_TEXT_TO_SPEECH' in os.environ:
    #     raise Exception('Env variable "GOOGLE_CREDENTIALS_TEXT_TO_SPEECH" is not set')


    # dialogflow_agent = DialogflowAgent(
    #     agent_name, 'me4', cred_file_incare_dialog)
    # cred_file_text_to_speech = os.environ['GOOGLE_CREDENTIALS_TEXT_TO_SPEECH']

    rospy.Subscriber("txt_send", String, lambda x: callback(x, playback_queue))

    rospy.Subscriber("wav_send", String, lambda x: callback_wav(x, playback_queue))

    rospy.Subscriber('/new_intent', tiago_msgs.msg.NewIntent, lambda x: callback_new_intent(x))
    

    data2_dir = data_dir + '/container'
    sc = SentencesContainer(data2_dir)
    say_as = SaySentenceActionServer(
        'rico_says', playback_queue, odm, sentence_dict, sc, cred_file_incare_dialog)

    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
        playback_queue.spin_once()
        rospy.sleep(0.1)


if __name__ == '__main__':
    #text = u'asdf**ńą'
    #text2 = text.replace('\n', ' ').replace('\t', ' ').replace('*', ' ').strip().lower()
    #text3 = text.replace(u'\n', u' ').replace(u'\t', u' ').replace(u'*', u' ').strip().lower()
    # print u'{}*{}'.format(text, 'qwer')
    # print u'{}*{}'.format(text3, 'qwer')
    # print u'{}*{}'.format(text2, 'qwer')
    # exit(0)
    listener()
