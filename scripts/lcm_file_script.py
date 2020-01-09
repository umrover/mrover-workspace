#!/usr/bin/python
import os
import json

lcm_dict = {}
lcm_dict['message_types'] = []

json_file = open('../base_station/gui/src/static/rover_msgs.json', 'w+')

for filename in os.listdir('../rover_msgs'):
    if filename.endswith('.lcm'):
        with open(os.path.join('../rover_msgs/' + filename), 'r') as f:
            #TODO: Consider replacing this with reading until we see 'struct'
            f.readline()
            f.readline()
            message_type_name = f.readline()[7:-3]
            print(message_type_name + "|")

            lcm_dict['message_types'].append(message_type_name)
            message_type_dict = []
            line = f.readline()

            while line.find('}') == -1:
                line = line.replace('  ', '')
                line = line.replace(';', '')
                line = line.replace("\n", '')
                line = line.replace("\r", '')
                line = line.replace("\t", '')
                line = line.split("//", 1)[0]
                
                #find out later how to process const variables with set values
                #ex. (in NavStatus.lcm)
                #const int8_t OFF = 0;

                if (line.find('=') == -1):
                    message_type_dict.append(line.split(' '))

                line = f.readline()

            lcm_dict[message_type_name] = message_type_dict

json.dump(lcm_dict, json_file, indent=4, sort_keys=True)
