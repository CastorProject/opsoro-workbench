import json

from channels.generic.websocket import AsyncWebsocketConsumer

from robot.commands import command_robot


class RobotControlSocket(AsyncWebsocketConsumer):
    __user = None
    __type = 'control'
    __group_name__ = ''

    async def websocket_connect(self, event):
        self.__user = self.scope['user']

        if not self.__user.is_authenticated:
            await self.close(1)
            return

        print("CONNECT REQUEST:", self.scope["path"], self.__user, event)

        # Add to all-user-group
        await self.channel_layer.group_add('all', self.channel_name)

        # update socket name according to path
        path = self.scope['path'].split('/')
        path = [a for a in path if a != '']  # remove all occurrences of ''
        if len(path) > 1:
            self.__type = path[-1]
        self.__group_name__ = ('%s-%s' % ('robot', self.__user))

        print("SET SOCKET NAME:", self.__group_name__)

        # Create group per user (app, virtual model, etc...)
        await self.channel_layer.group_add(self.__group_name__, self.channel_name)

        # Accept the connection request
        await self.accept()

    async def receive(self, text_data=None, bytes_data=None):
        print("RECEIVE MSG: ", text_data)
        if not self.__user.is_authenticated:
            return

        # text_data = event['text']

        if self.__type is 'control':
            msg = command_robot(self.__user, text_data)
            if msg is None:
                await self.channel_layer.group_send(
                    self.__group_name__,
                    {
                        'type': 'websocket.error',
                        'text': 'Invalid command'
                    },
                )
            else:
                msg_text = json.dumps(msg)
                if 'action' in msg:
                    if msg['action'] == 'data':
                        await self.channel_layer.group_send(
                            self.__group_name__,
                            {
                                'type': 'websocket.data',
                                'text': msg_text
                            },
                        )
                    elif msg['action'] == 'robot':
                        # group send or just send...
                        await self.channel_layer.group_send(
                            self.__group_name__,
                            {
                                'type': 'websocket.robot',
                                'text': msg_text
                            },
                        )

    async def websocket_robot(self, event):
        await self.send(text_data=event['text'])
        print("WS ROBOT", event)

    async def websocket_error(self, event):
        await self.send(text_data=event['text'])
        print("WS ERROR", event)

    async def websocket_data(self, event):
        await self.send(text_data=event['text'])
        print("WS DATA", event)

    async def disconnect(self, code):
        await self.channel_layer.group_discard(
            'all',
            self.channel_name
        )

        await self.channel_layer.group_discard(
            self.__group_name__,
            self.channel_name
        )
