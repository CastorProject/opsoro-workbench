from django.contrib.auth.decorators import login_required
from django.db.models import Q
from django.http import HttpResponse, HttpResponseRedirect
from django.shortcuts import render
from django.template import loader
from django.utils import timezone

# from documents.models import File
from opsoro.forms import NewsLetterForm
from robot.models import Robot
# from shop.forms import PreOrderForm
from shop.models import Product

from .forms import *
from .models import *

try:
    import simplejson as json
except ImportError:
    import json


def apps(request):
    # Get all categories that contain apps
    categories = Category.objects.exclude(apps__isnull=True)

    context = {'index': True, 'categories': categories, 'newsletter_form': NewsLetterForm()}

    template = loader.get_template('apps/index.html')
    return HttpResponse(template.render(context, request))


def preview(request):
    # prod = None
    # try:
    #     prod = Product.objects.filter(name__iexact="beta access")[0]
    # except Exception as e:
    #     print(e)
    #     pass

    # return render(request, 'preview.html', {'beta': prod, 'form': PreOrderForm(), 'newsletter_form': NewsLetterForm()})
    return render(request, 'preview.html', {'newsletter_form': NewsLetterForm()})


@login_required
def app(request, app_name):  # , file_name=""):
    # Check publish date
    try:
        # robot = get_object_or_404(Robot, owner=request.user.id)
        app = App.objects.get(Q(name__iexact=app_name) | Q(name__iexact=app_name.replace('_', ' ')))
        if not (request.user.is_staff or app.author == request.user):
            if app.date_publish >= timezone.now():
                return HttpResponseRedirect('/')
    except App.DoesNotExist:
        print('App "%s" does not exist' % app_name)
        return HttpResponseRedirect('/')

    # Retrieve app template
    try:
        template = loader.get_template('apps/' + app.formatted_name() + '/index.html')
    except Exception as e:
        print('Template error: ' + str(e))
        return HttpResponseRedirect('/')

    context = {'app': app,
               'data': {},
               'actions': {},
               'online': 1, }

    # newFile = False
    # file_name = None
    # if request.GET.get('f'):
    #     # if file_name:
    #     #     newFile = True
    file_name = request.GET.get('f', None)

    if file_name and len(file_name) > 1:
        context['actions'] = {'openfile': str(file_name)}
        #     try:
        #         file_obj = Document.objects.get(author=request.user, name=file_name.title())
        #         # if newFile:
        #         #     return HttpResponseRedirect('/apps/' + app_name + "/" + file_name)

    #     except Document.DoesNotExist:
    #         print('Doc does not exist')
    #         return HttpResponseRedirect('/app/' + app_name + '/')
    #
    # # Provide robot data if present
    # try:
    #     # robot = get_object_or_404(Robot, owner=request.user.id)
    #     # robot = Robot.objects.get(owner=request.user.id)
    #     # context['config'] = robot.config
    #     # context['emotions'] = robot.emotions
    #     pass
    # except Robot.DoesNotExist:
    #     pass

    return HttpResponse(template.render(context, request))


def feedback(request):
    if request.method == "POST":
        form = FeedbackForm(request.POST)
        if form.is_valid():
            # messages.add_message(request, messages.SUCCESS, 'You will be the first one to know, when the Kickstartercampaign launches!')
            cleaned_data = form.clean()
            po = Feedback()
            po.author = request.user
            if 'text' in cleaned_data:
                text = cleaned_data['text']
                po.text = text
            else:
                return HttpResponse(json.dumps({'success': False, 'message': 'Something went wrong.'}), content_type="application/json")

            if 'page' in cleaned_data:
                page = cleaned_data['page']
                po.page = page

            if 'category' in cleaned_data:
                category = cleaned_data['category']
                if category:
                    po.category = str(category)

            po.save()

            return HttpResponse(json.dumps({'success': True, 'message': 'Thank you for helping! We will look into it!'}), content_type="application/json")
        else:
            # messages.add_message(request, messages.ERROR, 'Something went wrong :/ ')
            return HttpResponse(json.dumps({'success': False, 'message': 'Something went wrong.'}), content_type="application/json")

    return render(request, '_feedback_modal.html', {'form': FeedbackForm()})
    # return redirect('/')

    # def form_valid(self, form):
    #     print('saaaaaaavve')
    #
    #     if self.messages.get("settings_updated"):
    #         messages.add_message(
    #             self.request,
    #             self.messages["settings_updated"]["level"],
    #             self.messages["settings_updated"]["text"]
    #         )
    #     return redirect(self.get_success_url())
