# -*- coding: utf-8 -*-
"""
ENPM661 Project 5, Spring 2020
Shelly Bagchi

Use this file to read in the exposure values for Scenario 2.
"""
import ast

def ReadDict():
    f = open("Total_Exposure.txt", "r")
    contents = f.read()
    exposure_dict = ast.literal_eval(contents)
    f.close()
    
    return exposure_dict


if __name__ == "__main__":
    exposure_dict = ReadDict()
    print(exposure_dict)