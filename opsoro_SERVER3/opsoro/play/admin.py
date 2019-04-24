from django.contrib import admin

from .models import *


class RelatedInline(admin.TabularInline):
    model = Category
    extra = 1


class AppAdmin(admin.ModelAdmin):
    inlines = (RelatedInline,)


# class FeedbackAdmin(admin.ModelAdmin):
#     inlines = (RelatedInline,)


admin.site.register(App)
admin.site.register(Category)
admin.site.register(Feedback)
