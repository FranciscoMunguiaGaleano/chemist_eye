from slack_bolt import App
from slack_bolt.adapter.socket_mode import SocketModeHandler
from slack_sdk import WebClient

import configparser
import logging

# load the tokens and signatures 
config = config = configparser.ConfigParser()
config.read("bot.ini")

# configure the tokens and secret signature
app_token = config["KEYS"]["AppToken"]
bot_token = config["KEYS"]["BotToken"]
signing_secret = config["KEYS"]["SlackSignature"]

# start the slack interface
app = App(token=bot_token, signing_secret=signing_secret)
client = WebClient(token=bot_token)

# start the logger
logger = logging.getLogger(__name__)
logging.basicConfig(filename='chemisteyebot.log',level=logging.DEBUG)

@app.event("app_mention")
def mention_handler(body, say):
    channel = body["event"]["channel"]  # Extract the channel ID
    say(channel)  # Send a message to the same channel
    
if __name__ == "__main__":
    handler = SocketModeHandler(app=app, app_token=app_token)
    handler.start()