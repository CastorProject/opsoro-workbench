import json
import math

from django.contrib.auth.decorators import login_required
from django.http import Http404, HttpResponse, HttpResponseRedirect
from django.shortcuts import get_object_or_404, render
from django.template import loader

from .models import Robot


# from django.contrib.auth.models import User


def constrain(n, minn, maxn): return max(min(maxn, n), minn)


@login_required
def index(request):
    template = loader.get_template('robot/index.html')

    data = {}
    try:
        robot = Robot.objects.filter(owner=request.user)[0]
        data = {'robot': robot.get_data}
    except Robot.DoesNotExist:
        raise Http404("You have no robot. Please go to the configurator app and build/save one.")

    return HttpResponse(template.render(data, request))


@login_required
def simple(request):
    template = loader.get_template('robot/simple.html')

    data = {}
    try:
        robot = Robot.objects.filter(owner=request.user)[0]
        data = {'robot': robot.get_data}
    except Robot.DoesNotExist:
        raise Http404("You have no robot. Please go to the configurator app and build/save one.")

    return HttpResponse(template.render(data, request))


@login_required
def config(request):
    try:
        config_data = request.POST.get('config_data')
    except:
        # print(config_data)
        raise Http404("Invalid data.")

    updated_values = {'config': config_data}
    obj, created = Robot.objects.update_or_create(owner=request.user, defaults=updated_values)
    # try:
    #     # robot = get_object_or_404(Robot, owner=request.user.id)
    #     robot = Robot.objects.get(owner=request.user.id)
    # except Robot.DoesNotExist:
    # 	if request.user.id is not None:
    #
    #
    #
    # 		name = models.CharField(max_length=20)
    # 	    owner = models.CharField(max_length=200, default='')
    #
    # 	    # saved as 1000x dof value (-1.0 <-> 1.0 = -1000 <-> 1000)
    # 	    dofs = models.CommaSeparatedIntegerField(
    # 	        max_length=200, default='0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
    #
    # 	    config = models.CharField(max_length=20000, default='{}')
    # 	    emotions = models.CharField(max_length=20000, default='{}')
    #
    #
    #
    #
    #     raise Http404(
    #         "You have no robot. Please go to the configurator app and build/save one.")
    # robot.config = config_data
    # robot.save()
    return HttpResponse('')
