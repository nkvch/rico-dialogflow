#!/usr/bin/env python
# encoding: utf8

import os
from DialogflowAgent import DialogflowAgent

agent_name = 'robot-rico-qrct'
cred_file_incare_dialog = os.environ['GOOGLE_CONVERSATIONAL_DIALOGFLOW']

audio_file_path = '/home/nkvch/tiago_public_ws/src/rcprg/dialogflow/data/container/sound_186868.wav'

agent = DialogflowAgent(agent_name, 'me4', cred_file_incare_dialog)

# with open(audio_file_path, 'rb') as audio_file:
#   input_audio = audio_file.read()

response = agent.repeat_text('pojedź do kuchni')

print response
