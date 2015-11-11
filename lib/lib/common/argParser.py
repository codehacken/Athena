########################################################
# argParser.py

#   This script handles parsing part of the image 
#             processing module
#      
#
#  Author : Nisha Pillai
#  Date : 11/02/2015
#
#
########################################################

import argparse

def argument_parse():
   argParse = argparse.ArgumentParser()
   argParse.add_argument("-i", "--image", help = "path to the image")
   args = vars(argParse.parse_args())
   return args


