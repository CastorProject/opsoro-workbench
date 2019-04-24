
from datetime import datetime, timedelta

from django.contrib.admin.views.decorators import staff_member_required
from django.contrib.auth.decorators import login_required, user_passes_test
from django.contrib.auth.models import User
from django.http import HttpResponse
from django.shortcuts import render
from django.template import loader
from django.utils import timezone

from play.models import Feedback
from shop.models import NewsLetter, PreOrder

from .models import Resource

try:
    import simplejson as json
except ImportError:
    import json


def is_allowed(user):
    return user.is_superuser or user.is_staff or user.groups.filter(name='Marketing').exists()


@login_required
@user_passes_test(is_allowed, redirect_field_name='')
def overview(request):

    # new:
    #   - users
    #   - orders
    #   - newsletters
    #   - feedback
    # usage graph
    this_week = timezone.now() - timedelta(days=7)
    orders = PreOrder.objects.filter(date__gte=this_week).order_by('-date')
    newsletters = NewsLetter.objects.filter(date__gte=this_week).order_by('-date')
    users = User.objects.filter(date_joined__gte=this_week).order_by('-date_joined')
    feedbacks = Feedback.objects.filter(date_created__gte=this_week).order_by('-date_created')

    context = {
        'usercount': str(users.count()) + '/' + str(User.objects.all().count()),
        'users': users,
        'newslettercount': str(newsletters.count()) + '/' + str(NewsLetter.objects.all().count()),
        'newsletters': newsletters,
        'preordercount': str(orders.count()) + '/' + str(PreOrder.objects.all().count()),
        'preorders': orders,
        'feedbacks': feedbacks,
    }

    template = loader.get_template('statistics.html')
    return HttpResponse(template.render(context, request))


@staff_member_required
def usage(request):
    # new:
    #   - users
    #   - orders
    #   - newsletters
    #   - feedback
    # usage graph
    this_week = timezone.now() - timedelta(days=7)
    usages = Resource.objects.filter(timestamp__gte=this_week).order_by('timestamp')

    context = {'usages': usages, 'cpudata': [], 'virtmemdata': [], 'swapmemdata': [], 'diskdata': [], 'timedata': []}

    for usage in usages:
        if usage.cpu:
            cpus = json.loads(usage.cpu)

            index = 0
            for cpu in cpus:
                if len(context['cpudata']) < len(cpus):
                    context['cpudata'].append([])
                if not context['cpudata'][index]:
                    context['cpudata'][index] = []
                context['cpudata'][index].append(cpu)
                index += 1
            # context['cpudata'].append(json.loads(usage.cpu)[0])
            # context['cpudata'].append(json.loads(usage.cpu)[1])
        if usage.virt_mem:
            context['virtmemdata'].append(json.loads(usage.virt_mem)['percent'])
        if usage.swap_mem:
            context['swapmemdata'].append(json.loads(usage.swap_mem)['percent'])
        if usage.disk:
            context['diskdata'].append(json.loads(usage.disk)['percent'])
        context['timedata'].append(usage.timestamp.strftime("%d/%m/%y-%H:%M"))

    # print(context)

    template = loader.get_template('usage.html')
    return HttpResponse(template.render(context, request))
