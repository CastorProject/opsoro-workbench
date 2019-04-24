import os
from functools import partial

from django.core.files.base import ContentFile

from documents.models import File
from play.models import App

get_path = partial(os.path.join, os.path.abspath(os.path.dirname(__file__)))


def _valid_parameters(appname, filename='dummy.ext', file_required=False):
    """
    Check the validity of given parameters.

    :param string appname:      current app name, to find the file of
    :param string filename:     name of the requested file
    :param bool file_required:  does the file needs to exist

    :return:        True if all parameters are valid.
    :rtype:         bool
    """
    if appname is None:
        return False

    if filename is None:
        return False

    # No path can be included in the filename
    if filename != os.path.basename(filename):
        return False

    # Filename should be at least 1 character long
    if len(os.path.splitext(filename)[0]) < 1:
        return False

    # # If a file is required, check if it exists
    # if file_required:
    #     if not os.path.exists(get_path('%s/%s' % (appname, filename))):
    #         return False

    return True


def filelist(user, appname, extension='.*', trim_ext=True):
    """
    Get the list of files of a certain app, filtered by an extension.

    :param string appname:      current app name, to find the files of
    :param string extension:    extension of requested files

    :return:        files of an app.
    :rtype:         list
    """
    if not _valid_parameters(appname, 'file' + extension):
        return None

    try:
        app = App.objects.get(name__iexact=appname)
    except App.DoesNotExist:
        return None

    file_list = File.objects.filter(author=user, app=app).order_by('name')
    files = []
    for fil in file_list:
        files.append({'name': fil.name, 'url': fil.data.url})

    context = {'files': files}

    return files


def read(user, appname, filename):
    """
    Read the data from a file.

    :param string appname:      current app name, to find the file of
    :param string filename:     name of the requested file

    :return:        file data.
    :rtype:         var
    """
    if not _valid_parameters(appname, filename, True):
        return None

    try:
        app = App.objects.get(name__iexact=appname)
    except App.DoesNotExist:
        return None

    try:
        file_obj = File.objects.get(author=user, name__iexact=os.path.splitext(filename)[0], app=app)
    except File.DoesNotExist:
        return None

    if not file_obj.data:
        return None

    return file_obj.data


def write(user, appname, filename, data):
    """
    Write data to a file.

    :param string appname:      current app name, to find the file of
    :param string filename:     name of the requested file
    :param var data:            data to write to the file

    :return:        True if write was successfull.
    :rtype:         bool
    """
    if not _valid_parameters(appname, filename, False):
        return False

    try:
        app = App.objects.get(name__iexact=appname)
    except App.DoesNotExist:
        return False

    updated_values = {'name': os.path.splitext(filename)[0]}
    try:
        obj, created = File.objects.update_or_create(
            author=user,
            name__iexact=os.path.splitext(filename)[0],
            extension=os.path.splitext(filename)[1][1:],
            defaults=updated_values,
        )

        # Add app to the document
        obj.app.add(app)

        # Save data
        obj.data.save(filename, ContentFile(data), save=True)

        obj.save()
    except Exception as e:
        print(e)
        return False

    return True


def delete(user, appname, filename):
    """
    Delete a file.

    :param string appname:      current app name, to find the file of
    :param string filename:     name of the requested file

    :return:        True if deletion was successfull.
    :rtype:         bool
    """
    if not _valid_parameters(appname, filename, True):
        return False

    try:
        app = App.objects.get(name__iexact=appname)
    except App.DoesNotExist:
        return False

    try:  # Check and load the file if it exists
        file_obj = File.objects.get(author=user, name__iexact=filename, app=app)
    except:
        return False

    # Delete file object
    file_obj.delete()

    return True
