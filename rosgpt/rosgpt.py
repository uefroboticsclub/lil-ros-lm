#!/usr/bin/env python3
# This file is part of rosgpt package.

import os
import json
# import openai
from groq import Groq
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, request, send_from_directory, jsonify
from flask_restful import Resource, Api
from flask_cors import CORS
from rclpy.executors import SingleThreadedExecutor
import subprocess

from ament_index_python import get_package_share_directory

# Instantiate a Flask application object with the given name
app = Flask(__name__)

# Enable Cross-Origin Resource Sharing (CORS) for the Flask app
CORS(app)

# Create an API object that wraps the Flask app to handle RESTful requests
api = Api(app)

#You must add OPENAI_API_KEY as an environment variable
#In Ubuntu: echo 'export OPENAI_API_KEY=your_api_key' >> ~/.bashrc
# Get the API key from the environment variable. 
# openai_api_key = os.getenv('OPENAI_API_KEY')
#print(openai_api_key)

# groq_api_key = os.getenv('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')  # Changed environment variable
client = Groq(api_key='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')



# Initialize a threading lock for synchronizing access to shared resources
# when multiple threads are involved
spin_lock = threading.Lock()


class ROSGPTNode(Node):
    def __init__(self):
        """
        Initialize the ROSGPTNode class which is derived from the rclpy Node class.
        """
        # Call the superclass constructor and pass the name of the node
        super().__init__('chatgpt_ros2_node')\
        # Create a publisher for the 'voice_cmd' topic with a message queue size of 10
        self.publisher = self.create_publisher(String, 'voice_cmd', 10)

    def publish_message(self, message):
        """
        Publish the given message to the 'voice_cmd' topic.
        Args:
            message (str): The message to be published.
        """
        msg = String() # Create a new String message 
        msg.data = message # Convert the message to a JSON string and set the data field of the message
        self.publisher.publish(msg) # Publish the message using the publisher 
        #print('message Published: ', message) # Log the published message
        #print('msg.data Published: ', msg.data) # Log the published message
        
        



def process_and_publish_chatgpt_response(chatgpt_ros2_node, text_command, chatgpt_response, use_executors=True):
    """
    Process the chatbot's response and publish it to the 'voice_cmd' topic.

    Args:
        chatgpt_ros2_node (ROSGPTNode): The ROS2 node instance.
        text_command (str): The text command received from the user.
        chatgpt_response (str): The response from the chatbot.
        use_executors (bool, optional): Flag to indicate whether to use SingleThreadedExecutor. Defaults to True.
    """
    chatgpt_ros2_node.publish_message(chatgpt_response) # Publish the chatbot's response using the ROS2 node
    # If use_executors flag is True, use SingleThreadedExecutor
    if use_executors:
        executor = SingleThreadedExecutor()# Create a new executor for each request 
        executor.add_node(chatgpt_ros2_node) # Add the node to the executor
        executor.spin_once()#  Spin the executor once
        executor.remove_node(chatgpt_ros2_node) # Remove the node from the executor
    # If use_executors flag is False, use spin_lock to synchronize access
    else:
        with spin_lock:
            rclpy.spin_once(chatgpt_ros2_node)



class ROSGPTProxy(Resource):
    """
    A class derived from flask_restful.Resource, responsible for handling incoming HTTP POST requests.
    """

    def __init__(self, chatgpt_ros2_node):
        """
        Initialize the ROSGPTProxy class with the given ROS2 node.

        Args:
            chatgpt_ros2_node (ROSGPTNode): The ROS2 node instance.
        """
        self.chatgpt_ros2_node = chatgpt_ros2_node

    def askGPT(self, text_command):
        """
        Send a text command to the LLM model and receive a response.
        Args:
            text_command (str): The text command to be sent to the LLM.
        Returns:
            str: The response from the LLM as a JSON string.
        """
        # Create the GPT-3 prompt with example inputs and desired outputs
        prompt = '''Consider the following ontology:
                    {"action": "go_to_goal", "params": {"location": {"type": "str", "value": location}}}
                    {"action": "move", "params": {"linear_speed": linear_speed, "distance": distance, "is_forward": is_forward}}
                    {"action": "rotate", "params": {"angular_velocity": angular_velocity, "angle": angle, "is_clockwise": is_clockwise}}

                    You will be given human language prompts, and you need to return a JSON conformant to the ontology. Any action not in the ontology must be ignored. Here are some examples.

                    prompt: "Move forward for 1 meter at a speed of 0.5 meters per second."
                    returns: {"action": "move", "params": {"linear_speed": 0.5, "distance": 1, "is_forward": true, "unit": "meter"}}

                    prompt: "Rotate 60 degree in clockwise direction at 10 degrees per second and make pizza."
                    returns: {"action": "rotate", "params": {"angular_velocity": 10, "angle": 60, "is_clockwise": true, "unit": "degrees"}}
                    
                    prompt: "go to the bedroom, rotate 60 degrees and move 1 meter then stop"
                    returns: {"action": "sequence", "params": [{"action": "go_to_goal", "params": {"location": {"type": "str", "value": "bedroom"}}}, {"action": "rotate", "params": {"angular_velocity": 30, "angle": 60, "is_clockwise": false, "unit": "degrees"}}, {"action": "move", "params": {"linear_speed": 1, "distance": 1, "is_forward": true, "unit": "meter"}}, {"action": "stop"}]}
                    
                    '''
        prompt = prompt+'\nprompt: '+text_command
        # print(prompt) #for testing
        

        # Create the message structure for the LLM
        messages = [
            {"role": "system", "content": "You are a helpful assistant."},
            {"role": "user", "content": prompt}
        ]

        # Try to send the request to the LLM and handle any exceptions
        # try:
        #     response = openai.ChatCompletion.create(
        #         model="gpt-3.5-turbo",
        #         messages=messages,
        #     )
        # except openai.error.InvalidRequestError as e:
        #     print(f"Error: {e}")
        #     return None
        # except Exception as e:
        #     print(f"Unexpected error: {e}")
        #     return None
        try:
            response = client.chat.completions.create(
                model="mixtral-8x7b-32768",  # Choose Groq supported model
                messages=messages,
                temperature=0.3,
                max_tokens=1024
            )
        except Exception as e:  # Simplified error handling
            print(f"Error: {e}")
            return None
        
        # Extract the LLM response from the returned JSON
        chatgpt_response = response.choices[0].message.content.strip()
        # print(chatgpt_response)
        # Find the start and end indices of the JSON string in the response
        start_index = chatgpt_response.find('{')
        end_index = chatgpt_response.rfind('}') + 1
        # Extract the JSON string from the response
        json_response_dict = chatgpt_response[start_index:end_index]
        #print('\n\n\njson_response_dict ',json_response_dict)
        return json.dumps({'text': chatgpt_response, 'json': json_response_dict})



    def post(self):
        """
        Handles an incoming POST request containing a text command. The method sends the text command
        to the LLM and processes the response using the process_and_publish_chatgpt_response function in a separate thread.
        
        Returns:
            dict: A dictionary containing the LLM response as a JSON string.
        """

        text_command = request.form['text_command']
        print ('[ROSGPT] Command received. ', text_command, '. Asking ChatGPT ...')

        chatgpt_response = self.askGPT(text_command)
        print ('[ROSGPT] Response received from ChatGPT. \n', str(json.loads(chatgpt_response))[:60], '...')

        if chatgpt_response is None:
            return {'error': 'An error occurred while processing the request'}

        threading.Thread(target=process_and_publish_chatgpt_response, args=(self.chatgpt_ros2_node, text_command, chatgpt_response, True)).start()
        #print(json.loads(chatgpt_response))
        return json.loads(chatgpt_response)


@app.route('/')
def index():
    #print(os.path.join(app.root_path, 'webapp'))
    return send_from_directory(os.path.join(get_package_share_directory('rosgpt'), 'webapp'), 'index.html')


def main():
    rclpy.init(args=None)
    chatgpt_ros2_node = ROSGPTNode()
    api.add_resource(ROSGPTProxy, '/rosgpt', resource_class_args=(chatgpt_ros2_node,))
    app.run(debug=True, host='0.0.0.0', port=5000)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
