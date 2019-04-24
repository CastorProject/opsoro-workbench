from django.conf.urls import url

from . import views

app_name = 'analytics'
urlpatterns = [
    url(r'^$', views.overview, name='overview'),
    url(r'^usage/', views.usage, name='simple'),
    # url(r'^mails/', views.mails, name='config'),

]
