import os
import uuid

from django.contrib.auth.models import User
from django.db import models
from django.db.models.signals import post_save
from django.dispatch import receiver

from robot.models import Robot


# Extend user

def avatar_upload(instance, filename):
    ext = filename.split(".")[-1]
    filename = "%s.%s" % (uuid.uuid4(), ext)
    return os.path.join("avatars", filename)


class Profile(models.Model):
    user = models.OneToOneField(User, on_delete=models.CASCADE)

    address = models.CharField(max_length=125, blank=True)
    postal = models.CharField(max_length=15, blank=True)
    city = models.CharField(max_length=50, blank=True)
    country = models.CharField(max_length=50, blank=True)
    birth_date = models.DateField(null=True, blank=True)
    newsletter = models.BooleanField(default=True)

    avatar = models.ImageField(upload_to=avatar_upload, blank=True)

    smiles = models.PositiveIntegerField(default=0)
    # smiles = models.PositiveIntegerField(default=0)

    date_created = models.DateTimeField(auto_now_add=True, null=True, blank=True)
    ip_created = models.GenericIPAddressField(null=True, blank=True)

    date_last_login = models.DateTimeField(null=True, blank=True)
    ip_last_login = models.GenericIPAddressField(null=True, blank=True)

    def __str__(self):
        return self.user.first_name + ' ' + self.user.last_name

    @receiver(post_save, sender=User)
    def create_user_profile(sender, instance, created, **kwargs):
        # print(request.META['REMOTE_ADDR'])
        if created:
            Profile.objects.create(user=instance)

            # Give new user existing robot
            try:
                robot = Robot.objects.filter(pk=1)[0]

                # Change user to new user
                robot.owner = instance

                modulelinks = None
                gridlink = None
                skinlink = None
                expressions = None

                if hasattr(robot, 'modulelink'):
                    modulelinks = robot.modulelink.all()
                if hasattr(robot, 'modulelink'):
                    gridlink = robot.gridlink
                if hasattr(robot, 'skinlink'):
                    skinlink = robot.skinlink
                if hasattr(robot, 'expression'):
                    expressions = robot.expression.all()

                # Save new instance
                robot.pk = None
                robot.save()

                for modlink in modulelinks:
                    armlink = None
                    if hasattr(modlink, 'armlink'):
                        armlink = modlink.armlink
                    modlink.pk = None
                    modlink.robot = robot
                    modlink.save()
                    if armlink:
                        armlink.pk = None
                        armlink.module = modlink
                        armlink.save()

                if gridlink:
                    gridlink.pk = None
                    gridlink.robot = robot
                    gridlink.save()

                if skinlink:
                    skinlink.pk = None
                    skinlink.robot = robot
                    skinlink.save()

                if expressions:
                    for expression in expressions:
                        dofs = expression.dofs.all()
                        expression.pk = None
                        expression.robot = robot
                        expression.save()
                        expression.dofs.set(dofs)

            except Robot.DoesNotExist:
                pass

    @receiver(post_save, sender=User)
    def save_user_profile(sender, instance, **kwargs):
        instance.profile.save()


class FriendLink(models.Model):
    user_1 = models.ForeignKey(User, related_name='friendlink_user1', on_delete=models.CASCADE)
    user_2 = models.ForeignKey(User, related_name='friendlink_user2', on_delete=models.CASCADE)

    accepted = models.BooleanField(default=True)

    date_created = models.DateTimeField(auto_now_add=True, null=True, blank=True)
    date_updated = models.DateTimeField(auto_now=True, null=True, blank=True)
    date_accepted = models.DateTimeField(auto_now=True, null=True, blank=True)

    def __str__(self):
        return self.user_1.name + ' + ' + self.user_2.name

    def save(self):
        super(FriendLink, self).save()

        # Send mail
