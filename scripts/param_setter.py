#!/usr/bin/env python3

import rclpy
import os
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter
from rclpy.parameter import ParameterValue, ParameterType

ROS_DOMAIN_ID :int = int(os.environ.get('ROS_DOMAIN_ID',0))

class ParamSetter(Node):
    def __init__(self, server_name:str)->None:
        super().__init__('parameter_setter')

        self.cli = self.create_client(SetParameters, '/' + server_name + '/set_parameters')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = SetParameters.Request()

    def set_param(self, param_name, param_value)->None:
        if isinstance(param_value, float):
            val = ParameterValue(double_value=param_value, type=ParameterType.PARAMETER_DOUBLE)
        elif isinstance(param_value, int):
            val = ParameterValue(integer_value=param_value, type=ParameterType.PARAMETER_INTEGER)
        elif isinstance(param_value, str):
            val = ParameterValue(string_value=param_value, type=ParameterType.PARAMETER_STRING)
        elif isinstance(param_value, bool):
            val = ParameterValue(bool_value=param_value, type=ParameterType.PARAMETER_BOOL)
        self.req.parameters = [Parameter(name=param_name, value=val)]
        self.future = self.cli.call_async(self.req)
        
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                try:
                    response: SetParameters.Response = self.future.result()

                    if response.results[0].successful:
                        self.get_logger().info(f'Successful set param : {param_name}: {param_value}')
                        return True
                except Exception as e:
                    pass
                return False
            
def main(args=None):

    rclpy.init(domain_id=ROS_DOMAIN_ID)

    node = ParamSetter(server_name = 'stm_serial_node')
    node.declare_parameter('wheelparam', 12000)
    node.set_param('wheelparam', node.get_parameter('wheelparam').value)
  
    node.destroy_node()  
    rclpy.shutdown()            

if __name__ == '__main__':
    main()    