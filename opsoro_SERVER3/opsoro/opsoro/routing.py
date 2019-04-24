from channels.auth import AuthMiddlewareStack
from channels.routing import ProtocolTypeRouter, URLRouter
from django.conf.urls import url

from play.consumers import RobotControlSocket

application = ProtocolTypeRouter({
    # (http->django views is added by default)
    'websocket': AuthMiddlewareStack(
        (
            URLRouter([
                url(r"^ws/$", RobotControlSocket),
                url(r"^ws/robot/$", RobotControlSocket),
            ])
        )
    ),
})
