from django.contrib import admin

from .models import *


class ModuleLinkInline(admin.TabularInline):
    model = ModuleLink
    extra = 0


class GridLinkInline(admin.TabularInline):
    model = GridLink
    extra = 0


class SkinLinkInline(admin.TabularInline):
    model = SkinLink
    extra = 0


class ExpressionInline(admin.TabularInline):
    model = Expression
    extra = 0


class ExpressionAdmin(admin.ModelAdmin):
    list_filter = ('robot', 'symbol_code', 'poly_index')
    list_display = ('robot', 'name', 'symbol_code', 'short_code', 'poly_index')


class RobotAdmin(admin.ModelAdmin):
    list_display = ('owner', 'name')
    inlines = [ModuleLinkInline, GridLinkInline, SkinLinkInline, ExpressionInline]


class DofServoInline(admin.TabularInline):
    model = DofServo
    extra = 0


class ModuleAdmin(admin.ModelAdmin):
    list_display = ('name', 'svg', 'javascript', 'width', 'height')
    inlines = [DofServoInline]


class GridAdmin(admin.ModelAdmin):
    list_display = ('name', 'svg', 'width', 'height')


class SkinAdmin(admin.ModelAdmin):
    list_display = ('name', 'svg', 'width', 'height')


class ArmAdmin(admin.ModelAdmin):
    list_display = ('name', 'svg', 'width', 'height')


class ArmLinkInline(admin.TabularInline):
    model = ArmLink
    extra = 0


class ModuleLinkAdmin(admin.ModelAdmin):
    list_filter = ('robot', 'module')
    list_display = ('robot', 'module', 'name', 'x', 'y', 'rotation')
    inlines = [ArmLinkInline]


admin.site.register(Robot, RobotAdmin)
admin.site.register(Module, ModuleAdmin)
admin.site.register(Grid, GridAdmin)
admin.site.register(Skin, SkinAdmin)
admin.site.register(Arm, ArmAdmin)
admin.site.register(ModuleLink, ModuleLinkAdmin)
admin.site.register(Expression, ExpressionAdmin)
admin.site.register(DofValue)
