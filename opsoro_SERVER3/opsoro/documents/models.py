import os

from django.contrib.auth.models import User
from django.db import models
from django.dispatch import receiver
from django.utils import timezone

from opsoro.helpers import delete_file_on_delete, delete_file_on_save
from play.models import App


def user_directory_path(instance, filename):
    # file will be uploaded to MEDIA_ROOT/user_<id>/<filename>
    return 'documents/files/{0}/{1}'.format(instance.author.id, filename)


# Create your models here.
class File(models.Model):
    author = models.ForeignKey(User, related_name='files', on_delete=models.CASCADE, default=0)
    name = models.CharField(max_length=100)
    extension = models.CharField(max_length=6, default='txt')

    date_created = models.DateTimeField(auto_now_add=True, null=True, blank=True)
    date_changed = models.DateTimeField(auto_now=True, null=True, blank=True)

    app = models.ManyToManyField(App)

    data = models.FileField(upload_to=user_directory_path)

    def __str__(self):
        return self.name + '.' + self.extension

    # def display_text_file(self):
    #     try:
    #         with open(self.data.path) as fp:
    #             return fp.read()  # .replace('\n', '<br/>')
    #     except:
    #         return ''


class SoundCollection(models.Model):
    author = models.ForeignKey(User, related_name='soundcollections', on_delete=models.CASCADE)
    name = models.CharField(max_length=100)

    public = models.BooleanField(default=False)

    date_created = models.DateTimeField(auto_now_add=True, null=True, blank=True)
    date_changed = models.DateTimeField(auto_now=True, null=True, blank=True)

    def __str__(self):
        return self.name


class Sound(models.Model):
    author = models.ForeignKey(User, on_delete=models.CASCADE)
    collection = models.ForeignKey(SoundCollection, related_name='sounds', on_delete=models.CASCADE)

    name = models.CharField(max_length=100)
    extension = models.CharField(max_length=6, default='wav')

    date_created = models.DateTimeField(auto_now_add=True, null=True, blank=True)
    date_changed = models.DateTimeField(auto_now=True, null=True, blank=True)

    data = models.FileField(upload_to='sounds/')

    def __str__(self):
        return self.name + '.' + self.extension


class TTS(models.Model):
    author = models.ForeignKey(User, on_delete=models.CASCADE)
    # espeak "" -v nl+f2 -p 50 -g 10 -s 200 -w test.wav
    name = models.CharField(max_length=100)
    # -v
    language = models.CharField(max_length=10, default='en')
    gender = models.CharField(max_length=1, default='m')
    gender_index = models.PositiveSmallIntegerField(default=2)

    # -p
    pitch = models.PositiveSmallIntegerField(default=50)
    # -g
    word_gap = models.PositiveSmallIntegerField(default=10)
    # -s
    speed = models.PositiveSmallIntegerField(default=175)

    date_created = models.DateTimeField(auto_now_add=True, null=True, blank=True)
    date_changed = models.DateTimeField(auto_now=True, null=True, blank=True)

    # -w
    data = models.FileField(upload_to='tts/', null=True, blank=True)

    def __str__(self):
        return self.name


@receiver(models.signals.pre_save, sender=File)
@receiver(models.signals.pre_save, sender=Sound)
@receiver(models.signals.pre_save, sender=TTS)
def pre_save_file(sender, instance, **kwargs):
    delete_file_on_save(instance)


@receiver(models.signals.post_delete, sender=File)
@receiver(models.signals.post_delete, sender=Sound)
@receiver(models.signals.post_delete, sender=TTS)
def post_delete_file(sender, instance, **kwargs):
    delete_file_on_delete(instance)

    #
    #
    #
    # # These two auto-delete files from filesystem when they are unneeded:
    # @receiver(models.signals.post_delete, sender=MediaFile)
    # def auto_delete_file_on_delete(sender, instance, **kwargs):
    #     """Deletes file from filesystem
    #     when corresponding `MediaFile` object is deleted.
    #     """
    #     if instance.file:
    #         if os.path.isfile(instance.file.path):
    #             os.remove(instance.file.path)
    #
    # @receiver(models.signals.pre_save, sender=MediaFile)
    # def auto_delete_file_on_change(sender, instance, **kwargs):
    #     """Deletes file from filesystem
    #     when corresponding `MediaFile` object is changed.
    #     """
    #     if not instance.pk:
    #         return False
    #
    #     try:
    #         old_file = MediaFile.objects.get(pk=instance.pk).file
    #     except MediaFile.DoesNotExist:
    #         return False
    #
    #     new_file = instance.file
    #     if not old_file == new_file:
    #         if os.path.isfile(old_file.path):
    #             os.remove(old_file.path)
