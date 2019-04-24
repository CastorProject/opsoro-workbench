from django.conf.urls import url

from . import views

app_name = 'documents'
urlpatterns = [
    # /documents/
    url(r'^$', views.index, name='index'),
    # /
    # url(r'^view/(?P<file_name>[\w]+)/$', views.file_view, name='file_view'),
    url(r'^data/(?P<app_name>[\w]+)/$', views.file_data, name='file_data'),
    # url(r'^app/(?P<app_name>[\w]+)/$', views.app_files, name='app_files'),
    url(r'^save/(?P<app_name>[\w]+)/$', views.file_save, name='file_save'),
    url(r'^delete/(?P<app_name>[\w]+)/$', views.file_delete, name='file_delete'),
    url(r'^list/', views.app_files, name='file_list'),
    # url(r'^type/(?P<type>[\w]+)/$', views.app_files, name='app_files'),
]
