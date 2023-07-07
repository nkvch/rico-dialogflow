#!/usr/bin/env python
# encoding: utf8

#
# Copyright 2018 Picovoice Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import argparse
import os
import struct
import sys
from datetime import datetime
from threading import Thread

import numpy as np
import pyaudio
import soundfile
import wave
import time

# packages for VAD
import webrtcvad
import collections
from array import array
from struct import pack
from scipy.signal import butter, lfilter, lfilter_zi

has_ros = True
try:
	import rospy
	from std_msgs.msg import String, Bool
except:
	has_ros = False


# start recording after that many voiced frames
THR_VOICED = 5

# stop recording after that many unvoiced/silence frames
THR_UNVOICED = 10

# stop recording after that many seconds
THR_TIME = 5

# minimum volume (power) for voiced frames
THR_POWER = 100

# do normalize the output wave
DO_NORMALIZE = True

# device id
DEVICE_ID = 1




sys.path.append(os.path.join(os.path.dirname(__file__), '../pkgs/porcupine/binding/python'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../pkgs/porcupine/resources/util/python'))

#import porcupine
from porcupine import Porcupine
from util import *


from multiprocessing.queues import Queue

DATA_DIR=os.path.join(os.path.dirname(__file__), '../data')
SAMPLE_RATE_REC=16000
SAMPLE_RATE_WORK=16000

def butter_bandpass(lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return b, a


def record_to_file(path, data, sample_width, rate):
    "Records from the microphone and outputs the resulting data to 'path'"
    # sample_width, data = record()
    data = pack('<' + ('h' * len(data)), *data)
    wf = wave.open(path, 'wb')
    wf.setnchannels(1)
    wf.setsampwidth(sample_width)
    wf.setframerate(rate)
    wf.writeframes(data)
    wf.close()


def normalize(snd_data):
    "Average the volume out"
    MAXIMUM = 32767  # 16384
    times = float(MAXIMUM) / max(abs(i) for i in snd_data)
    r = array('h')
    for i in snd_data:
        r.append(int(i * times))
    return r


class PorcupineDemo(Thread):
    """
    Demo class for wake word detection (aka Porcupine) library. It creates an input audio stream from a microphone,
    monitors it, and upon detecting the specified wake word(s) prints the detection time and index of wake word on
    console. It optionally saves the recorded audio into a file for further review.
    """

    def __init__(
            self,
            library_path,
            model_file_path,
            keyword_file_paths,
            sensitivities,
            input_device_index=None,
            output_path=None):

        """
        Constructor.

        :param library_path: Absolute path to Porcupine's dynamic library.
        :param model_file_path: Absolute path to the model parameter file.
        :param keyword_file_paths: List of absolute paths to keyword files.
        :param sensitivities: Sensitivity parameter for each wake word. For more information refer to
        'include/pv_porcupine.h'. It uses the
        same sensitivity value for all keywords.
        :param input_device_index: Optional argument. If provided, audio is recorded from this input device. Otherwise,
        the default audio input device is used.
        :param output_path: If provided recorded audio will be stored in this location at the end of the run.
        """

        super(PorcupineDemo, self).__init__()

        self._library_path = library_path
        self._model_file_path = model_file_path
        self._keyword_file_paths = keyword_file_paths
        self._sensitivities = sensitivities
        self._input_device_index = input_device_index

        self.play_name = ''
        self.play_id = 0 

        self.recorded_frames = Queue()

        self._output_path = output_path
        if self._output_path is not None:
            self._recorded_frames = []

        self.__activate_vad_received = False
        self.__vad_enabled = True
        if has_ros:
            print("Opening ros")
            rospy.init_node('vad', anonymous=True)
            print("Connecting to publisher")
            self.pub = rospy.Publisher('wav_send', String, queue_size=10)

            self.sub_activate_vad = rospy.Subscriber('/activate_vad', Bool, self.__activate_vad_callback)

            self.sub_vad_enabled = rospy.Subscriber('vad_enabled', Bool, self.__vad_enabled_callback)
        else:
            self.pub = None
        print("Done")

    def __activate_vad_callback(self, data):
        print 'activate_vad_received'
        self.__activate_vad_received = True

    def __vad_enabled_callback(self, data):
        if data.data == True:
            self.__vad_enabled = True
        elif data.data == False:
            self.__vad_enabled = False
        else:
            raise

    def get_next_frame(self):
        if self.play_name == '':
            output = np.zeros(512, dtype=np.int16).tostring()
            self.play_id = 0
            return output

        output = self.sounds[self.play_name][self.play_id * 512 : (self.play_id + 1) * 512]
        self.play_id = self.play_id + 1
        if len(output) < 512:
            output = np.pad(output, (0, 512-len(output)), 'constant', constant_values=(0,0))
            self.play_name=''

        output = output.tostring()
        return output

        

    def audio_callback(self, in_data, frame_count, time_info, status):
        decoded_block = np.fromstring(in_data, 'Int16')
#        decoded_block = decoded_block.repeat(2, axis=0)
        in_data = pack('<' + ('h' * len(decoded_block)), *decoded_block)
        filtered_block, self.zi = lfilter(self.b, self.a, decoded_block, zi=self.zi)
        filtered_block = filtered_block.astype(np.int16)
        chunk_to_analyze = pack('<' + ('h' * len(filtered_block)), *filtered_block)


        if self.play_name == '':
            self.recorded_frames.put({'orig': in_data, 'filt': chunk_to_analyze})
        
        output = self.get_next_frame()
        return output, pyaudio.paContinue

    def quickplay(self, pa, data, wf):
        out_stream = pa.open(format =
            pa.get_format_from_width(wf.getsampwidth()),
            channels = wf.getnchannels(),
            rate = wf.getframerate(),
            output = True)
        out_stream.write(data)

    def run(self):
        """
         Creates an input audio stream, initializes wake word detection (Porcupine) object, and monitors the audio
         stream for occurrences of the wake word(s). It prints the time of detection for each occurrence and index of
         wake word.
         """

        num_keywords = len(self._keyword_file_paths)

        keyword_names = list()
        for x in self._keyword_file_paths:
            keyword_names.append(os.path.basename(x).replace('.ppn', '').replace('_compressed', '').split('_')[0])

        print('listening for:')
        for keyword_name, sensitivity in zip(keyword_names, self._sensitivities):
            print('- %s (sensitivity: %f)' % (keyword_name, sensitivity))

        porcupine = None
        pa = None
        audio_stream = None

        # open the file for reading.
        wf = wave.open(os.path.join(DATA_DIR, 'snd_on.wav'), 'rb')
        wg = wave.open(os.path.join(DATA_DIR, 'snd_off.wav'), 'rb')

        try:
            porcupine = Porcupine(
                library_path=self._library_path,
                model_file_path=self._model_file_path,
                keyword_file_paths=self._keyword_file_paths,
                sensitivities=self._sensitivities)

            print(porcupine.frame_length)

            FILT_LOW = 400
            FILT_HIGH = 4000
            self.b, self.a = butter_bandpass(FILT_LOW, FILT_HIGH, SAMPLE_RATE_WORK, order=5)
            self.zi = lfilter_zi(self.b, self.a)

            pa = pyaudio.PyAudio()
            audio_stream = pa.open(
                #rate=porcupine.sample_rate,
                rate=SAMPLE_RATE_REC,
                channels=1,
                format=pyaudio.paInt16,
                input=True,
                output=True,
                frames_per_buffer=porcupine.frame_length,
                input_device_index=self._input_device_index,
                output_device_index=self._input_device_index,
                stream_callback=self.audio_callback)

            # open stream based on the wave object which has been input.

            wav_data = wf.readframes(-1)
            wav2_data = wg.readframes(-1)
            self.sounds = {}
            self.sounds['on'] = np.fromstring(wav_data, 'Int16')
            self.sounds['off'] = np.fromstring(wav2_data, 'Int16')

            while True:
                if has_ros and rospy.is_shutdown():
                    break

                #pcm = audio_stream.read(porcupine.frame_length, False)
                pcm = self.recorded_frames.get()['orig']
                pcm = struct.unpack_from("h" * porcupine.frame_length, pcm)

                if self._output_path is not None:
                    self._recorded_frames.append(pcm)

                result = porcupine.process(pcm)
                if self.__vad_enabled and ( (num_keywords == 1 and result) or self.__activate_vad_received ):
                    print('[%s] detected keyword' % str(datetime.now()))
                    #self.quickplay(pa, wav_data, wf)
                    self.play_name ='on'
                    self.runvad()
                    self.play_name='off'
                    #self.quickplay(pa, wav2_data, wf)
                    self.__activate_vad_received = False


                elif num_keywords > 1 and result >= 0:
                    print('[%s] detected %s' % (str(datetime.now()), keyword_names[result]))
                    out_stream.write(wav_data)

        except KeyboardInterrupt:
            print('stopping ...')
        finally:
            if porcupine is not None:
                porcupine.delete()

            if audio_stream is not None:
                audio_stream.close()

            if pa is not None:
                pa.terminate()

            if self._output_path is not None and len(self._recorded_frames) > 0:
                recorded_audio = np.concatenate(self._recorded_frames, axis=0).astype(np.int16)
                soundfile.write(self._output_path, recorded_audio, samplerate=porcupine.sample_rate, subtype='PCM_16')

    _AUDIO_DEVICE_INFO_KEYS = ['index', 'name', 'defaultSampleRate', 'maxInputChannels']


    def runvad(self):
        CHANNELS = 1
        RATE = SAMPLE_RATE_WORK
        CHUNK_DURATION_MS = 30       # supports 10, 20 and 30 (ms)
        PADDING_DURATION_MS = 1500   # 1 sec jugement
        CHUNK_SIZE = int(RATE * CHUNK_DURATION_MS / 1000)  # chunk to read
        CHUNK_BYTES = CHUNK_SIZE * 2  # 16bit = 2 bytes, PCM
        NUM_PADDING_CHUNKS = int(PADDING_DURATION_MS / CHUNK_DURATION_MS)
        # NUM_WINDOW_CHUNKS = int(240 / CHUNK_DURATION_MS)
        NUM_WINDOW_CHUNKS = int(400 / CHUNK_DURATION_MS)  # 400 ms/ 30ms  ge
        #NUM_WINDOW_CHUNKS = int(384 / CHUNK_DURATION_MS)
        NUM_WINDOW_CHUNKS_END = NUM_WINDOW_CHUNKS * 5

        START_OFFSET = int(NUM_WINDOW_CHUNKS * CHUNK_DURATION_MS * 0.5 * RATE)

        vad = webrtcvad.Vad(3)

        got_a_sentence = False

        ring_buffer = collections.deque(maxlen=NUM_PADDING_CHUNKS)
        triggered = False
        voiced_frames = []
        ring_buffer_flags = [0] * NUM_WINDOW_CHUNKS
        ring_buffer_index = 0

        ring_buffer_flags_end = [1] * NUM_WINDOW_CHUNKS_END
        ring_buffer_index_end = 0
        buffer_in = ''

        raw_data = array('h')
        index = 0
        start_point = 0
        StartTime = time.time()
        TimeUse = 0
        print("* recording: ")

	
        cancelled = False

        num_unv = 0
        ignore = 5
        while not got_a_sentence and TimeUse <= THR_TIME:
            if not self.__vad_enabled:
                cancelled = True
                break

            data = self.recorded_frames.get()
            if ignore > 0:
                ignore = ignore - 1
                continue
            chunk = data['orig']
            chunk_to_analyze = data['filt'][0:960]
            decoded_block = np.fromstring(chunk_to_analyze, 'Int16')
            orig_block = np.fromstring(chunk, 'Int16')
 
            # add WangS
            raw_data.extend(array('h', chunk))
            index += len(decoded_block)
            TimeUse = time.time() - StartTime

            power = np.mean(np.abs(decoded_block))
            sp_flag = vad.is_speech(chunk_to_analyze, RATE) 
            #active = sp_flag and power > 500
            active = power > THR_POWER
            if active:
                num_unv = 0
            else:
                num_unv = num_unv + 1


            #sys.stdout.write('1'*int(power/100)+' ' + str(power) + '          \r' if sp_flag else '_'*int(power/100)+ ' ' + str(power) + '             \r')
            sys.stdout.write('O' if active else '-')

            ring_buffer_flags[ring_buffer_index] = 1 if active else 0
            ring_buffer_index += 1
            ring_buffer_index %= NUM_WINDOW_CHUNKS

            ring_buffer_flags_end[ring_buffer_index_end] = 1 if active else 0
            ring_buffer_index_end += 1
            ring_buffer_index_end %= NUM_WINDOW_CHUNKS_END

            # start point detection
            if not triggered:
                ring_buffer.append(chunk)
                num_voiced = sum(ring_buffer_flags)
                if num_voiced > THR_VOICED:
                    sys.stdout.write('<<')
                    ring_buffer_flags_end = [1] * NUM_WINDOW_CHUNKS_END
                    triggered = True
                    start_point = index - 512 * 8  # start point
                    # voiced_frames.extend(ring_buffer)
                    ring_buffer.clear()
            # end point detection
            else:
                # voiced_frames.append(chunk)
                ring_buffer.append(chunk)
                num_unvoiced = NUM_WINDOW_CHUNKS_END - sum(ring_buffer_flags_end)
                if num_unv > THR_UNVOICED or TimeUse > THR_TIME:
                    sys.stdout.write('>>')
                    triggered = False
                    got_a_sentence = True

            sys.stdout.flush()

        sys.stdout.write('\n')
        if cancelled:
            print '* vad cancelled'
        else:
            print("* done recording")
            if got_a_sentence:
                got_a_sentence = False

                # write to file
                raw_data.reverse()
                for index in range(start_point):
                    raw_data.pop()
                raw_data.reverse()

                if DO_NORMALIZE:
                    raw_data = normalize(raw_data)

                now = datetime.now() # current date and time

                fname = now.strftime("/tmp/%m-%d-%Y-%H-%M-%S") + ".wav"
                record_to_file(fname, raw_data, 2, RATE)
                print("Saved to " + fname)
                if has_ros:
                    self.pub.publish(fname)




    @classmethod
    def show_audio_devices_info(cls):
        """ Provides information regarding different audio devices available. """

        pa = pyaudio.PyAudio()

        for i in range(pa.get_device_count()):
            info = pa.get_device_info_by_index(i)
            print(', '.join("'%s': '%s'" % (k, str(info[k])) for k in cls._AUDIO_DEVICE_INFO_KEYS))

        pa.terminate()


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument('--keywords', help='comma-separated list of default keywords (%s)' % ', '.join(KEYWORDS))

    parser.add_argument('--keyword_file_paths', help='comma-separated absolute paths to keyword files')

    parser.add_argument('--library_path', help="absolute path to Porcupine's dynamic library", default=LIBRARY_PATH)

    parser.add_argument('--model_file_path', help='absolute path to model parameter file', default=MODEL_FILE_PATH)

    parser.add_argument('--sensitivities', help='detection sensitivity [0, 1]', default=0.5)

    parser.add_argument('--input_audio_device_index', help='index of input audio device', type=int, default=None)

    parser.add_argument(
        '--output_path',
        help='absolute path to where recorded audio will be stored. If not set, it will be bypassed.')

    parser.add_argument('--show_audio_devices_info', action='store_true')

    PorcupineDemo.show_audio_devices_info()

   # args,unknown = parser.parse_args()

   # if args.show_audio_devices_info:
   #     PorcupineDemo.show_audio_devices_info()
   # else:
   #     if args.keyword_file_paths is None:
   #         if args.keywords is None:
   #             raise ValueError('either --keywords or --keyword_file_paths must be set')

   #         keywords = [x.strip() for x in args.keywords.split(',')]
    keywords=['hey pico']       
    if all(x in KEYWORDS for x in keywords):
        keyword_file_paths = [KEYWORD_FILE_PATHS[x] for x in keywords]
    else:
        raise ValueError('selected keywords are not available by default. available keywords are: %s' % ', '.join(KEYWORDS))
    
    sensitivities = [0.99]

    PorcupineDemo(
            library_path=LIBRARY_PATH,
            model_file_path=MODEL_FILE_PATH,
            keyword_file_paths=keyword_file_paths,
            sensitivities=sensitivities,
            output_path=None,
            # input_device_index=2
        ).run()


if __name__ == '__main__':
    main()
