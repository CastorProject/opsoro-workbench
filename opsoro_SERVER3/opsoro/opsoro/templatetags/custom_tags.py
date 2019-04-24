import datetime

import pytz
from django import template

register = template.Library()


@register.inclusion_tag('_app_button.html')
def app_button(app):
    app_enabled = (app.date_publish <= datetime.datetime.now(pytz.utc))
    return {
        'app': app,
        'app_enabled': app_enabled,
        'app_background': app.color if app_enabled else 'gray_lighter',
        'app_href': '/app/' + app.formatted_name() if app_enabled else '#',
        'app_class': app.formatted_name() + ('' if app_enabled else ' disabled'),
        'opsoro_author': (app.author.username == 'OPSORO'),
    }


@register.filter(name='cut')
def cut(value, arg):
    """Removes all values of arg from the given string"""
    return value.replace(arg, '')
