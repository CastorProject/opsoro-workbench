from django.conf.urls import url

from . import views

app_name = 'robot'
urlpatterns = [
    url(r'^$', views.index, name='index'),
    url(r'^simple/', views.simple, name='simple'),
    # url(r'^(?P<robot_id>[0-9]+)$', views.index, name='index'),
    # url(r'^dofs/', views.dofs, name='dofs'),
    # url(r'^emotion/', views.emotion, name='emotion'),
    url(r'^config/', views.config, name='config'),
]
