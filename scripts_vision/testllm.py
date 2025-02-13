#!/usr/bin/env python

import ollama
import os
import cv2
import time

IMGS_TEMP_1_PATH = os.path.join(os.path.dirname(__file__), '..', 'temp', 'imgone.jpg')
IMGS_TEMP_2_PATH = os.path.join(os.path.dirname(__file__), '..', 'temp', 'imgtwo.jpg')
TEST_PATH = "imgs/image.png"
def query_llm(img_path):
    try:
        if not os.path.exists(img_path):
            print(f"Image file not found: {img_path}")
            return None

        with open(img_path, 'rb') as img_file:
            image_data = img_file.read()
            print(f"Querying LLM with image data of size: {len(image_data)} bytes")

            response = ollama.chat(
                model='llava:7b',
                messages=[
                    {
                        'role': 'user',
                        'content': 'If anyone in the image, can you tell if everyone that is facing the camera is wearing glasses? Only reply by writing Yes or No',
                        'images': [image_data],  # Pass binary data here
                    }
                ],
            )

        # Extract and log the response content
        message_content = response.get('message', {}).get('content', '').strip()
        print(f"Content: {response}")
        print(f"LLM Response Content: {message_content}")
        return message_content
    except Exception as e:
        print(f"Error querying LLM: {e}")
        return None



if __name__ == '__main__':
    answer = str(query_llm(IMGS_TEMP_2_PATH)).upper()
    answer = str(query_llm(TEST_PATH)).upper()
