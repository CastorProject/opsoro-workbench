from django.contrib import admin

# Register your models here.
from .models import File, Sound, SoundCollection


class FileAdmin(admin.ModelAdmin):
    list_filter = ('app', 'extension', 'author')
    list_display = ('name', 'author', 'extension', 'data', 'date_created', 'date_changed')


class SoundAdmin(admin.ModelAdmin):
    list_filter = ('collection', 'extension', 'author')
    list_display = ('name', 'collection', 'author', 'extension', 'data', 'date_created', 'date_changed')


class SoundInline(admin.TabularInline):
    model = Sound
    extra = 0


class SoundCollectionAdmin(admin.ModelAdmin):
    list_filter = ('author',)
    list_display = ('name', 'public', 'author', 'date_created', 'date_changed')
    inlines = [SoundInline]


admin.site.register(File, FileAdmin)
admin.site.register(SoundCollection, SoundCollectionAdmin)
admin.site.register(Sound, SoundAdmin)
