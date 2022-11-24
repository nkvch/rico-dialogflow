import dialogflow_v2 as dialogflow


class DialogflowAgent:
    def __init__(self, project_id, session_id, cred_file):
        self.project_id = project_id
        self.session_id = session_id
        self.cred_file = cred_file
        self.contexts_client = dialogflow.ContextsClient.from_service_account_file(
            cred_file)
        self.session_client = dialogflow.SessionsClient.from_service_account_file(
            cred_file)
        self.intents_client = dialogflow.IntentsClient.from_service_account_file(
            cred_file)
        self.intents_parent = self.intents_client.project_agent_path(
            project_id)
        self.session = self.session_client.session_path(project_id, session_id)
        audio_encoding = dialogflow.enums.AudioEncoding.AUDIO_ENCODING_LINEAR_16
        voice = dialogflow.types.VoiceSelectionParams(
            name="pl-PL-Wavenet-B"
        )
        synthesize_speech_config = dialogflow.types.SynthesizeSpeechConfig(
            pitch=-5,
            speaking_rate=0.8,
            voice=voice)
        output_audio_config = dialogflow.types.OutputAudioConfig(
            audio_encoding=audio_encoding,
            synthesize_speech_config=synthesize_speech_config
        )
        language_code = 'pl'
        sample_rate_hertz = 16000
        input_audio_config = dialogflow.types.InputAudioConfig(
            audio_encoding=audio_encoding, language_code=language_code,
            sample_rate_hertz=sample_rate_hertz)

        self.config = {
            'audio_encoding': audio_encoding,
            'voice': voice,
            'synthesisze_speech_config': synthesize_speech_config,
            'output_audio_config': output_audio_config,
            'language_code': language_code,
            'sample_rate_hertz': sample_rate_hertz,
            'input_audio_config': input_audio_config
        }

    def repeat_text(self, text):
        repeat_data = dialogflow.types.Struct()
        repeat_data.update({'phrase': text})

        event_input = dialogflow.types.EventInput(
            name='trigger-repeat', parameters=repeat_data, language_code=self.config['language_code'])
        query_input = dialogflow.types.QueryInput(event=event_input)

        response = self.session_client.detect_intent(
            session=self.session, query_input=query_input,
            output_audio_config=self.config['output_audio_config']
        )

        return response

    def detect_intent_audio(self, input_audio):
        query_input = dialogflow.types.QueryInput(
            audio_config=self.config['input_audio_config'])

        response = self.session_client.detect_intent(
            session=self.session, query_input=query_input,
            input_audio=input_audio,
            output_audio_config=self.config['output_audio_config']
        )

        return response

    def detect_intent_text(self, text):
        text_input = dialogflow.types.TextInput(
            text=text, language_code=self.config['language_code'])
        query_input = dialogflow.types.QueryInput(text=text_input)

        response = self.session_client.detect_intent(
            session=self.session, query_input=query_input,
            output_audio_config=self.config['output_audio_config']
        )

        return response

# parameters looks like [{ 'name': ..., 'prompt': '' }, { 'name': ..., 'prompt': '' }]
    def create_intent(self, name, trigger_phrases, autoresponses, parameters=[], default_parameters=[]):
        print name, trigger_phrases, autoresponses, parameters, default_parameters

        training_phrases = list(map(lambda phrase: dialogflow.types.Intent.TrainingPhrase(
            parts=[dialogflow.types.Intent.TrainingPhrase.Part(text=unicode(phrase, 'utf-8'))]), trigger_phrases))

        print 'training phrases ok'

        messages = list(map(lambda response_msg: dialogflow.types.Intent.Message(
            text=dialogflow.types.Intent.Message.Text(text=[unicode(response_msg, 'utf-8')])), autoresponses))

        print 'messages ok'

        parameters = list(map(lambda param: dialogflow.types.Intent.Parameter(display_name=unicode(param['name'], 'utf-8'), prompts=[unicode(param['prompt'], 'utf-8')], mandatory=True, entity_type_display_name='@sys.any'), parameters))

        print 'parameters ok'

        default_parameters = list(map(lambda param: dialogflow.types.Intent.Parameter(display_name=unicode(param['name'], 'utf-8'), mandatory=True, entity_type_display_name='@sys.any', value=param['value']), default_parameters))

        print 'default parameters ok'

        intent = dialogflow.types.Intent(
            display_name=unicode(name, 'utf-8'),
            training_phrases=training_phrases,
            messages=messages,
            parameters=parameters + default_parameters,
            priority=1000000
        )

        print 'intent ok'

        response = self.intents_client.create_intent(
            self.intents_parent, intent)
        
        print 'all ok'

        return response
