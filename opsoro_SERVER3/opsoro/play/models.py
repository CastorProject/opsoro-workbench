import os
import shutil
import uuid

from django.contrib.auth.models import User
from django.db import models
from django.utils import timezone


# Create your models here.
class App(models.Model):
    name = models.CharField(max_length=50, unique=True)
    author = models.ForeignKey(User, on_delete=models.SET_DEFAULT, default=1)

    icon = models.CharField(max_length=50, default='fa-cube')
    color = models.CharField(max_length=16, default='green')

    difficulty = models.PositiveSmallIntegerField(default=0)

    date_created = models.DateTimeField(auto_now_add=True, null=True, blank=True)
    date_publish = models.DateTimeField('publish date', default=timezone.now)

    def __str__(self):
        return self.name

    def name_lower(self):
        return self.name.lower()

    def full_name(self):
        return self.name.title()

    def formatted_name(self):
        return self.name.lower().replace(' ', '_')

    def save(self, *args, **kwargs):
        if not self.pk:  # the object doesn't have PK until the very first save
            super(App, self).save(*args, **kwargs)

            # Add app immediatly to the uncategorized app list
            obj, created = Category.objects.update_or_create(index=9999)
            obj.apps.add(self)
            obj.save()

            # Create static folder
            static_basedir = 'opsoro/static/apps/' + self.formatted_name()
            if not os.path.exists(static_basedir):
                os.makedirs(static_basedir)

            # Create template folder
            template_basedir = 'play/templates/apps/' + self.formatted_name()
            if not os.path.exists(template_basedir):
                os.makedirs(template_basedir)

            # Copy empty template if no file is present
            if not os.path.isfile(template_basedir + '/index.html'):
                shutil.copyfile('play/templates/apps/template.html', template_basedir + '/index.html')

            # print('New app1!!')
            # Test2.objects.create(test=self)
        else:
            super(App, self).save(*args, **kwargs)

    def delete(self, *args, **kwargs):
        # first, delete the files
        # Static folder
        print(self.name.lower())
        basedir = 'opsoro/static/apps/' + self.formatted_name()
        if os.path.exists(basedir):
            shutil.rmtree(basedir, ignore_errors=False, onerror=None)

        # Template folder
        basedir = 'play/templates/apps/' + self.formatted_name()
        if os.path.exists(basedir):
            shutil.rmtree(basedir, ignore_errors=False, onerror=None)

        # now, delete the object
        super(App, self).delete(*args, **kwargs)


class Category(models.Model):
    index = models.PositiveSmallIntegerField(default=0)
    name = models.CharField(max_length=50, default='...', unique=True)
    apps = models.ManyToManyField(App, blank=True)

    def __str__(self):
        return self.name

    def save(self, *args, **kwargs):
        super(Category, self).save(*args, **kwargs)

        if self.index == 9999:
            return

        try:
            apps = []
            for cat in Category.objects.exclude(index=9999):
                apps.extend(cat.apps.all())

            other_apps = []
            for app in App.objects.all():
                if app not in apps:
                    other_apps.append(app)

            obj, created = Category.objects.update_or_create(index=9999)
            obj.apps = other_apps
            obj.save()

        except Exception as e:
            print(e)


class Feedback(models.Model):
    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    author = models.ForeignKey(User, default=1, on_delete=models.CASCADE)

    page = models.CharField(max_length=200, null=True, blank=True)

    date_created = models.DateTimeField(auto_now_add=True, null=True, blank=True)
    date_update = models.DateTimeField(auto_now=True, null=True, blank=True)

    CATEGORY = (
        ('B', 'Bug'),
        ('I', 'Idea'),
        ('H', 'Help'),
    )
    category = models.CharField(max_length=1, choices=CATEGORY, null=True, blank=True)
    STATUS = (
        ('O', 'Open'),
        ('C', 'Checking'),
        ('I', 'Ignore'),
        ('T', 'Todo'),
        ('D', 'Done'),
    )
    status = models.CharField(max_length=1, choices=STATUS, default='O')

    text = models.CharField(max_length=1000, unique=True)

    def __str__(self):
        return self.author.username + ' - ' + self.category


class Vote(models.Model):
    author = models.ForeignKey(User, on_delete=models.SET_DEFAULT, default=1)

    smile = models.BooleanField(default=False)
    # smiles = models.PositiveIntegerField(default=0)
    # smiles = models.PositiveIntegerField(default=0)

    date_created = models.DateTimeField(auto_now_add=True, null=True, blank=True)
    date_publish = models.DateTimeField('publish date', default=timezone.now)

    def __str__(self):
        return self.author.username + ': ' + str(self.smile)


class FeedbackVote(models.Model):
    feedback = models.ForeignKey(Feedback, related_name='votelink', on_delete=models.CASCADE)
    vote = models.OneToOneField(Vote, related_name='votelink', on_delete=models.CASCADE)

    def __str__(self):
        return str(self.feedback.id) + ': (' + str(self.vote) + ')'
