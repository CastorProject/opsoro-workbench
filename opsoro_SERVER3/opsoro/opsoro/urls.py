from django.conf import settings
from django.conf.urls import include, url
from django.conf.urls.static import static
from django.contrib import admin
from django.views.generic import TemplateView

from opsoro.views import *
from shop import views as shop_views

from . import views

urlpatterns = [
    # url(r"^$", TemplateView.as_view(template_name="homepage.html"), name="home"),

    url(r'^', include('play.urls', namespace='play')),
    url(r'^$', shop_views.index, name='home'),
    url(r"^admin/", admin.site.urls),

    url(r"^account/password/$", views.password, name="account_password"),
    url(r"^account/settings/$", SettingsView.as_view(), name="account_settings"),
    url(r"^account/signup/$", SignupView.as_view(), name="account_settings"),
    url(r"^account/login/$", LoginView.as_view(), name="account_settings"),
    url(r"^account/password/reset/$", PasswordResetView.as_view(), name="account_settings"),
    url(r"^account/delete/$", DeleteView.as_view(), name="account_delete"),

    url(r"^account/orders/$", views.index, name="orders"),
    url(r'^account/orders(?P<orderId>\w+)/$', views.index, name="order_detail"),
    url(r'^account/orders/confirm(?P<orderId>\w+)/$', views.confirm_received, name="confirm_received"),
    url(r"^account/", include("account.urls")),
    url(r"^account/social/accounts/$", TemplateView.as_view(template_name="account/social.html"), name="account_social_accounts"),
    url(r"^account/social/", include("social_django.urls", namespace="social")),


    url(r"^announcements/", include("pinax.announcements.urls", namespace="pinax_announcements")),

    # url(r'^play/', include('play.urls', namespace='play')),
    url(r'^robot/', include('robot.urls', namespace='robot')),
    url(r'^docs/', include('documents.urls', namespace='documents')),
    url(r'^shop/', include('shop.urls', namespace='shop')),
    url(r"^analytics/", include('analytics.urls', namespace='analytics')),

    url(r"^forum/", include('forums.urls', namespace='forums'), name='forums'),
    # url(r"^teams/", include("teams.urls", namespace="teams"), name='teams'),

]

urlpatterns += static(settings.MEDIA_URL, document_root=settings.MEDIA_ROOT)
