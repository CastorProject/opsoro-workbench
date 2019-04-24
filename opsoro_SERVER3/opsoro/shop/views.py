from account.models import EmailAddress
from django.contrib import messages
from django.contrib.auth.models import AnonymousUser
from django.http import HttpResponse, HttpResponseRedirect
from django.shortcuts import redirect, render, render_to_response
from django.template import RequestContext, loader

from opsoro.forms import NewsLetterForm
from shop.forms import *
from shop.models import *

try:
    import simplejson as json
except ImportError:
    import json


def index(request):

    products = Product.objects.filter(active=True).order_by('pk')

    return render(request, 'shop.html', {'products': products, 'form': PreOrderForm(), 'newsletter_form': NewsLetterForm()})


def order(request):
    products = Product.objects.filter(active=True)

    if request.method == "POST":
        form = OrderPaymentForm(request.user, request.POST)
        if form.is_valid():
            messages.add_message(request, messages.SUCCESS, 'Your order has been placed')
            return HttpResponseRedirect('/account/orders/')
        else:
            messages.add_message(request, messages.ERROR, 'Something went wrong :/ ')

    initial = {}
    if not request.user.is_anonymous():
        primary_email_address = EmailAddress.objects.get_primary(request.user)
        if primary_email_address:
            initial['user_id'] = request.user.id
            initial['first_name'] = request.user.first_name
            initial['last_name'] = request.user.last_name
            initial['address'] = request.user.profile.address
            initial['postal'] = request.user.profile.postal
            initial['city'] = request.user.profile.city
            initial['country'] = request.user.profile.country

    return render(request, 'shop.html', {'products': products, 'form': OrderPaymentForm(request.user), 'newsletter_form': NewsLetterForm()})


def pre_order(request):

    if request.method == "POST":
        form = PreOrderForm(request.POST)
        if form.is_valid():
            # messages.add_message(request, messages.SUCCESS, 'You will be the first one to know, when the Kickstartercampaign launches!')
            return HttpResponse(json.dumps({'success': True, 'message': 'Thank you for pre-ordering! You will be the first to know, when our Kickstartercampaign launches!'}), content_type="application/json")
        else:
            # messages.add_message(request, messages.ERROR, 'Something went wrong :/ ')
            return HttpResponse(json.dumps({'success': False, 'message': 'Something went wrong.'}), content_type="application/json")

    return redirect('/')


def newsletter(request):

    if request.method == "POST":
        form = NewsLetterForm(request.POST)
        if form.is_valid():
            messages.add_message(request, messages.SUCCESS, 'We will keep you posted!')
            # return HttpResponse(json.dumps({'success': True, 'message': 'Thank you for pre-ordering! We will keep you posted!'}))
        else:
            messages.add_message(request, messages.ERROR, 'Something went wrong :/')
            # return HttpResponse(json.dumps({'success': False, 'message': 'Something went wrong.'}))

    return redirect('/')
