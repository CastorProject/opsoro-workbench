from django.conf.urls import url
from django.views.generic import TemplateView

from . import views

app_name = 'shop'
urlpatterns = [
    url(r'^$', views.index, name='index'),
    url(r"^test/$", views.index, name='order'),
    url(r'^pre-order/$', views.pre_order, name="pre_order"),
    url(r'^newsletter/$', views.newsletter, name="newsletter"),
]
