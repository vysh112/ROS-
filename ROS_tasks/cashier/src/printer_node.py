#!/usr/bin/env python3

import rospy
from cashier.msg import Bill
from std_msgs.msg import Float32
from std_msgs.msg import String

last_bill = None
inventory_param = '/inventory'
income_param = '/income'

def bill_callback(data):
    global last_bill
    last_bill = data

def print_last_bill():
    global last_bill

    rospy.loginfo("Previous Bill:\n%s", last_bill)
    inventory = rospy.get_param(inventory_param, 100)
    income = rospy.get_param(income_param, 0)
    rospy.loginfo("Current inventory: %f", inventory)
    rospy.loginfo("Current income: %f", income)

def main():
    rospy.init_node('printer', anonymous=True)
    rospy.Subscriber('bill_topic', Bill, bill_callback)

    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        user_input = input()
        
        if last_bill is not None:
            print_last_bill()
        rate.sleep()

if __name__ == '__main__':
    main()
