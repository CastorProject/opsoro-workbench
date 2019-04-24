from django.contrib import admin

from .models import *


# Register your models here.


class ProfileAdmin(admin.ModelAdmin):

    list_display = ('user', 'birth_date', 'country')


admin.site.register(Profile, ProfileAdmin)
