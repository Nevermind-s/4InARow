#!/usr/bin/env python2
"""
Don't forget to run roscore and
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600
"""
from __future__ import (absolute_import, division, print_function,
                        unicode_literals)
from builtins import (bytes, dict, int, list, object, range, str, ascii, chr,
                      hex, input, next, oct, open, pow, round, super, filter,
                      map, zip)
from math import (sqrt, acos, atan, degrees, hypot, pi)
import time

# pylint: disable=E0401
from std_msgs.msg import UInt16
import rospy

# List of available uArm operations
OPERAND = {
    'lower_arm': 1,
    'upper_arm': 2,
    'base_rot':  3,
    'pump_on':   4,
    'pump_off':  5,
    'reset':     6,
    'get_value': 7,  # not implemented
    'head_rot':  8,
    'mv':        9,
    'elev1':	10,
    'elev2':	11,
    'elev3':    12
}


def uarm_init():
    ctx = {'uarm': None,
		   'elev': None,
           'rate': None}
    rospy.init_node('raspi_commander', anonymous=True)
    ctx['uarm'] = rospy.Publisher('movements', UInt16, queue_size=10)
    ctx['elev'] = rospy.Publisher('elevator',  UInt16, queue_size=10)
    ctx['rate'] = rospy.Rate(1)
    return ctx


def encode_command(op, arg):
    """
    This function formats the instruction as an unsigned 16bit integer with
    the 8 MSB as opcode and the 8 LSB as argument.
    """
    return (op & 0xFF) << 8 | (arg & 0xFF)


def loop(ctx):
    """
    This is a debug function, ignore it.
    It turns the pump on and off at the frequency specified at rospy.Rate() in
    uarm_init().
    """
    def gen_turns():
        turn = True
        while True:
            yield turn
            turn = not turn
    turn = gen_turns()

    while not rospy.is_shutdown():
        msg = encode_command(OPERAND['pump_on']
                             if next(turn) else
                             OPERAND['pump_off'], 0)
        rospy.loginfo(msg)
        ctx['uarm'].publish(msg)
        ctx['rate'].sleep()


def moveto(ctx, query):
    """
    Move the head of the arm to the desired position.
    """
    def translation(x, y, z):
        """
        Convert (x,y,z) coordinates to servo (alpha,beta,gamma) motor angles.
        This is a conversion from an empirical relationship.
        """
        base = atan(x/y) + pi/2
        x = hypot(x*1.125, y)
        c = hypot(x, z)
        alpha = atan(z/x) + acos((3696+c*c)/(320*c))
        gamma = acos((-c*c+47504)/(47360))
        psi = alpha + gamma - pi/2
        alpha = int(degrees(alpha))
        base = int(degrees(base))
        psi = int(degrees(psi))
        # also remove offset from angles
        return (base, alpha-37, 68-psi)

    angle = translation(int(query[1]), int(query[2]), int(query[3]))
    msg = encode_command(OPERAND['base_rot'], angle[0])
    ctx['uarm'].publish(msg)
    msg = encode_command(OPERAND['lower_arm'], angle[1])
    ctx['uarm'].publish(msg)
    msg = encode_command(OPERAND['upper_arm'], angle[2])
    ctx['uarm'].publish(msg)


def execute(ctx, msg):
    """
    General debug function to execute a raw command.
    It takes a String as input in `msg`, the String looks like `pump_on 1`.
    The command must always contains 2 portions separated by a space.
    Therefor an argument is mandatory even if it's meaningless.
    """
    query = msg.split(' ')
    if query[0] not in OPERAND:
        print("{} is not an valid command.".format(query[0]))
        return
    rospy.loginfo(msg)
    if OPERAND[query[0]] == 6:
        # RESET, because I'm lazy
        msg = encode_command(OPERAND['lower_arm'], 45)
        ctx['uarm'].publish(msg)
        time.sleep(1)
        
    if OPERAND[query[0]] == 9:
        # special case for moveto which is not implemented on the arm
        moveto(ctx, query)     
    elif 10 <= OPERAND[query[0]] <= 12:
        # special case for elevator commands
        msg = encode_command(OPERAND[query[0]]-9, int(query[1]))
        ctx['elev'].publish(msg)
    else:
        # every others operations are directly implemented on the arm
        msg = encode_command(OPERAND[query[0]], int(query[1]))
        ctx['uarm'].publish(msg)
    ctx['rate'].sleep()


if __name__ == '__main__':
    try:
        ctx = uarm_init()
        while True and not rospy.is_shutdown():
            print(">>> ", end='')
            stdin = raw_input()
            execute(ctx, stdin)
    except rospy.ROSInterruptException:
        print("Shutdown signal received.")
    except (KeyboardInterrupt, EOFError) as err:
        print(type(err))
