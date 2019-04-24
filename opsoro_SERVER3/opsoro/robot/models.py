import uuid

from django.contrib.auth.models import User
from django.db import models
from django.dispatch import receiver

from opsoro.helpers import delete_file_on_delete, delete_file_on_save


class DofValue(models.Model):
    tags = models.CharField(max_length=200, default='')
    index = models.SmallIntegerField(default=-1)
    value = models.FloatField(default=0)

    def __str__(self):
        return str(self.value) + ' - ' + self.tags


class Robot(models.Model):
    owner = models.ForeignKey(User, on_delete=models.CASCADE, default=0)
    name = models.CharField(max_length=50, default='Robot')
    token = models.UUIDField(default=uuid.uuid4, editable=False)

    dofs = models.ManyToManyField(DofValue, blank=True)

    def __str__(self):
        return self.owner.username + ': ' + self.name

    def get_data(self):
        robot_data = {'svg_urls': {}, 'js_urls': [], 'grid': {}, 'skin': {}, 'modules': []}

        if hasattr(self, 'skinlink'):
            if self.skinlink.skin.svg:
                robot_data['skin']['svg_url'] = self.skinlink.skin.svg.url
            robot_data['skin']['width'] = self.skinlink.skin.width
            robot_data['skin']['height'] = self.skinlink.skin.height
            robot_data['skin']['x_offset'] = self.skinlink.skin.x_offset
            robot_data['skin']['y_offset'] = self.skinlink.skin.y_offset

        if hasattr(self, 'gridlink'):
            if self.gridlink.grid.svg:
                robot_data['grid']['svg_url'] = self.gridlink.grid.svg.url
            robot_data['grid']['width'] = self.gridlink.grid.width
            robot_data['grid']['height'] = self.gridlink.grid.height

        if hasattr(self, 'modulelink'):
            for modlink in self.modulelink.all():
                mod = modlink.module
                robot_data['svg_urls'][str(mod.name)] = mod.svg.url

                if mod.javascript.url not in robot_data['js_urls']:
                    robot_data['js_urls'].append(mod.javascript.url)

                tmp_mod = {'size': {}, 'grid': {}, 'extra': {}, 'dofs': []}
                tmp_mod['name'] = str(modlink.name)
                tmp_mod['type'] = str(mod.name)
                tmp_mod['size']['width'] = mod.width
                tmp_mod['size']['height'] = mod.height
                tmp_mod['grid']['x'] = modlink.x
                tmp_mod['grid']['y'] = modlink.y
                tmp_mod['grid']['rotation'] = modlink.rotation

                if hasattr(modlink, 'armlink'):
                    tmp_mod['extra']['svg_url'] = modlink.armlink.arm.svg.url
                    tmp_mod['extra']['width'] = modlink.armlink.arm.width
                    tmp_mod['extra']['height'] = modlink.armlink.arm.height
                    tmp_mod['extra']['x_offset'] = modlink.armlink.arm.x_offset
                    tmp_mod['extra']['y_offset'] = modlink.armlink.arm.y_offset
                    tmp_mod['extra']['rotation_offset'] = modlink.armlink.rotation_offset

                for dof in mod.dof.all():
                    # tmp_dof = {'servo': {}}
                    tmp_dof = {}
                    tmp_dof['name'] = str(dof.name)
                    tmp_mod['dofs'].append(tmp_dof)

                robot_data['modules'].append(tmp_mod)
        return robot_data


class Module(models.Model):     # ADMIN ONLY
    name = models.CharField(max_length=50, default='Module')

    svg = models.FileField(upload_to='modules/svg/', null=True, blank=True)
    javascript = models.FileField(upload_to='modules/js/', null=True, blank=True)

    # Module size in mm
    width = models.PositiveSmallIntegerField(default=0)
    height = models.PositiveSmallIntegerField(default=0)

    def __str__(self):
        return self.name


class Dof(models.Model):
    module = models.ForeignKey(Module, related_name='dof', on_delete=models.CASCADE)
    name = models.CharField(max_length=50, default='dof')


class DofServo(Dof):    # ADMIN ONLY
    # Servo data
    servo_pin = models.SmallIntegerField(default=-1)
    servo_mid = models.PositiveSmallIntegerField(default=1500)
    servo_min = models.SmallIntegerField(default=0)
    servo_max = models.SmallIntegerField(default=0)

    def __str__(self):
        return self.module.name + ' ' + self.name

#
# class Dof(DofSpec):
#     # Servo data
#     pin = models.SmallIntegerField(default=-1)
#     servo_mid = models.PositiveSmallIntegerField(default=1500)
#
#     def __str__(self):
#         return self.name


class ModuleLink(models.Model):
    robot = models.ForeignKey(Robot, related_name='modulelink', on_delete=models.CASCADE)
    module = models.ForeignKey(Module, related_name='modulelink', on_delete=models.CASCADE)

    name = models.CharField(max_length=20, default='module', null=True, blank=True)

    # Position in grid holes
    x = models.PositiveSmallIntegerField(default=0)
    y = models.PositiveSmallIntegerField(default=0)
    # Rotation (0 - 360)
    rotation = models.PositiveSmallIntegerField(default=0)

    def __str__(self):
        return self.module.name + ' ' + self.name


class Grid(models.Model):       # ADMIN ONLY
    name = models.CharField(max_length=50, default='A4 (portrait)')

    # Grid size in mm
    width = models.PositiveSmallIntegerField(default=205)
    height = models.PositiveSmallIntegerField(default=293)

    svg = models.FileField(upload_to='grids/svg/', null=True, blank=True)

    def __str__(self):
        return self.name


class GridLink(models.Model):
    robot = models.OneToOneField(Robot, related_name='gridlink', on_delete=models.CASCADE)
    grid = models.ForeignKey(Grid, related_name='gridlink', on_delete=models.CASCADE)

    def __str__(self):
        return self.grid.name


class Skin(models.Model):       # ADMIN ONLY
    name = models.CharField(max_length=50, default='Robot')

    # Skin size in mm
    width = models.PositiveSmallIntegerField(default=205)
    height = models.PositiveSmallIntegerField(default=293)

    # # Offsets in mm and degrees
    x_offset = models.SmallIntegerField(default=0)
    y_offset = models.SmallIntegerField(default=0)

    svg = models.FileField(upload_to='skins/svg/', null=True, blank=True)

    def __str__(self):
        return self.name


class SkinLink(models.Model):
    robot = models.OneToOneField(Robot, related_name='skinlink', on_delete=models.CASCADE)
    skin = models.ForeignKey(Skin, related_name='skinlink', on_delete=models.CASCADE)

    def __str__(self):
        return self.skin.name


class Arm(models.Model):        # ADMIN ONLY
    name = models.CharField(max_length=50, default='Small servo arm')

    # Grid size in mm
    width = models.PositiveSmallIntegerField(default=32)
    height = models.PositiveSmallIntegerField(default=7)

    # Offsets in mm and degrees
    x_offset = models.SmallIntegerField(default=0)
    y_offset = models.SmallIntegerField(default=0)

    svg = models.FileField(upload_to='arms/svg/', null=True, blank=True)

    def __str__(self):
        return self.name


class ArmLink(models.Model):
    module = models.OneToOneField(ModuleLink, related_name='armlink', on_delete=models.CASCADE)
    arm = models.ForeignKey(Arm, related_name='armlink', on_delete=models.CASCADE)

    # Offsets in mm and degrees
    rotation_offset = models.SmallIntegerField(default=0)

    def __str__(self):
        return self.arm.name


class Expression(models.Model):
    robot = models.ForeignKey(Robot, related_name='expression', on_delete=models.CASCADE)

    name = models.CharField(max_length=50, default='smile')
    symbol_code = models.CharField(max_length=5, default='1f642', null=True, blank=True)
    short_code = models.CharField(max_length=50, default=':slight_smile:', null=True, blank=True)

    poly_index = models.SmallIntegerField(default=-1)
    dofs = models.ManyToManyField(DofValue, blank=True)

    def __str__(self):
        return self.robot.name + ' ' + self.name


@receiver(models.signals.pre_save, sender=Module)
@receiver(models.signals.pre_save, sender=Grid)
@receiver(models.signals.pre_save, sender=Arm)
def pre_save_file(sender, instance, **kwargs):
    if not instance.pk:
        return False
    try:
        delete_file_on_save(instance, instance.svg, type(instance).objects.get(pk=instance.pk).svg)
    except Exception:
        print("EXCEPTION", type(instance), instance.pk, "DOES NOT EXIST")
        return False
        pass

    if hasattr(instance, 'javascript'):
        delete_file_on_save(instance, instance.javascript, type(instance).objects.get(pk=instance.pk).javascript)


@receiver(models.signals.post_delete, sender=Module)
@receiver(models.signals.post_delete, sender=Grid)
@receiver(models.signals.post_delete, sender=Arm)
def post_delete_file(sender, instance, **kwargs):
    delete_file_on_delete(instance, instance.svg)
    if hasattr(instance, 'javascript'):
        delete_file_on_delete(instance, instance.javascript)

# class ExpressionLink(models.Model):
#     robot = models.OneToOneField(Robot, related_name='expressionlink', on_delete=models.CASCADE)
#     expression = models.ForeignKey(Expression, related_name='expressionlink', on_delete=models.CASCADE)
#
#     date = models.DateTimeField(auto_now=True, null=True, blank=True)
#
#     def __str__(self):
#         return self.robot.name + ' - ' + self.expression.name
