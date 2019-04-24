from django.conf.urls import include, url

urlpatterns = (
    url(r"^", include("forums.urls", namespace="forums")),
)
