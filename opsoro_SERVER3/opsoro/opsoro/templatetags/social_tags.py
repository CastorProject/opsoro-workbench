from django import template

register = template.Library()


@register.tag  # 12/04/2019 possible incompatible update fix
def user_can_disconnect(user_social_auth):
    return user_social_auth.allowed_to_disconnect(user_social_auth.user, user_social_auth.provider)
