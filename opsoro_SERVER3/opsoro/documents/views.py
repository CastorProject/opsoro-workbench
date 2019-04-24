import os

from django.conf import settings
from django.contrib.auth.decorators import login_required
from django.core.files import File
from django.http import Http404, HttpResponse, HttpResponseRedirect
from django.shortcuts import get_object_or_404, render
from django.template import loader

from documents.data import *
from documents.models import File
from play.models import App

try:
    import simplejson as json
except ImportError:
    import json

file_type_extensions = {'sound': ['.wav', '.mp3'],
                        'picture': ['.png', '.jpg', '.gif']}


# Create your views here.
@login_required
def index(request):
    try:
        file_list = File.objects.filter(author=request.user).order_by('extension')
        context = {'files': file_list}
    except:
        pass

    template = loader.get_template('documents/index.html')
    return HttpResponse(template.render(context, request))


@login_required
def app_files(request, app_name=None):
    if app_name is None:
        app_name = str(request.GET.get('a'))

    files = filelist(request.user, app_name)
    if files is None:
        pass

    context = {'files': files}

    template = loader.get_template('_files.html')
    return HttpResponse(template.render(context, request))


@login_required
def file_data(request, app_name):
    try:
        filename = str(request.GET.get('f').title())
        filename = os.path.splitext(filename)[0]
        # file_extension = str(request.GET.get['e'])
    except:
        print("Invalid file data.")
        return HttpResponse(json.dumps({'success': False, 'message': 'Invalid input data.'}))

    file_data = read(request.user, app_name, filename)

    return HttpResponse(file_data)


@login_required
def file_view(request, filename):
    # try:  # Check and load the file if it exists
    #     file_obj = File.objects.get(author=request.user, name=filename)
    # except File.DoesNotExist:
    #     return HttpResponseRedirect('/docs/')
    #
    # context = {'file_obj': file_obj}
    # template = loader.get_template('documents/file.html')
    # return HttpResponse(template.render(context, request))
    return HttpResponse('')


@login_required
def file_save(request, app_name):
   # return HttpResponseRedirect('/docs/')

    try:
        filename = str(request.POST.get('filename'))
        file_data = request.POST.get('data')
    except:
        print("Invalid input data.")
        return HttpResponse(json.dumps({'success': False, 'message': 'Invalid input data.'}))

    if filename is 'None' or file_data is None:
        print("Invalid data.", filename, file_data)
        return HttpResponse(json.dumps({'success': False, 'message': 'Invalid input data.'}))

    returnval = write(request.user, app_name, filename, file_data)

    return HttpResponse(json.dumps({'success': returnval}))


@login_required
def file_delete(request, app_name):

    try:
        filename = str(request.POST.get('filename'))
    except:
        print("Invalid input data.")
        return HttpResponse(json.dumps({'success': False, 'message': 'Invalid input data.'}))

    returnval = delete(request.user, app_name, filename)

    return HttpResponse(json.dumps({'success': returnval}))
