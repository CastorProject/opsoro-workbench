from django.conf.urls import include, url

from . import views

app_name = 'play'
urlpatterns = [
    # /apps/
    url(r'^$', views.apps, name='apps'),
    # url(r'^$', views.preview, name='preview'),
    # url(r'^$', views.index, name='index'),
    # /apps/circumplex
    # url(r'^apps/(?P<app_name>[\w]+)/$', views.app, name='app'),
    # url(r'^(?P<app_name>[\w]+)/files/show$', views.app_files,
    #     name='app_files'),
    # url(r'^(?P<app_name>[\w]+)/(?P<file_name>[\w]+)/files/show$',
    #     views.app_files,
    #     name='app_files'),
    # url(r'^apps/', views.apps, name='apps'),
    url(r'^feedback/$', views.feedback, name="feedback"),



    url(r'^app/(?P<app_name>[\w]+)/$', views.app, name='app'),

    # url(r'^(?P<app_name>[\w]+)/(?P<file_name>[\w]+)/$', views.app, name='app'),
]
