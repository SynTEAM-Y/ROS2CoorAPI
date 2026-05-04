#!/usr/bin/env python3
import time
import argparse
from Rosmaster_Lib import Rosmaster

def main():
    bot = Rosmaster(debug=False)
    bot.create_receive_threading()
    bot.set_car_type(bot.CARTYPE_X3_PLUS)

    # Built-in battery display effect is 6
    bot.set_colorful_effect(6, speed=255, parm=255)

if __name__ == "__main__":
    main()
