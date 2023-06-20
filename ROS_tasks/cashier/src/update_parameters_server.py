#!/usr/bin/env python3

import rospy
from cashier.srv import UpdateParameters

def handle_update_parameters(request):
    current_inventory = 0.0  # Placeholder value, update with actual inventory
    current_income = 0.0  # Placeholder value, update with actual income

    response = UpdateParametersResponse()
    response.current_inventory = current_inventory
    response.current_income = current_income
    response.success = True

    return response

def update_parameters_server():
    rospy.init_node('update_parameters_server')
    service = rospy.Service('update_parameters', UpdateParameters, handle_update_parameters)
    rospy.spin()

if __name__ == '__main__':
    update_parameters_server()
