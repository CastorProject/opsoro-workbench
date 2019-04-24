import os


# from documents.models import *
# from opsoro.models import *
# from play.models import *
# from robot.models import *
# from shop.models import *


def delete_file_on_save(sender, new_file=None, old_file=None):
    """
    Deletes file from filesystem when corresponding `sender` object is changed.
    """
    # If soundfile exists, remove before saving a new one
    if not sender.pk:
        return False

    if old_file and new_file:
        if not old_file == new_file:
            if os.path.isfile(old_file.path):
                os.remove(old_file.path)

        return

    try:
        old_file = type(sender).objects.get(pk=sender.pk).data
    except type(sender).DoesNotExist:
        return

    if hasattr(sender, 'file'):
        new_file = sender.data
        if not old_file == new_file:
            if os.path.isfile(old_file.path):
                os.remove(old_file.path)

    return


def delete_file_on_delete(sender, file=None):
    """
    Deletes file from filesystem when corresponding `sender` object is deleted.
    """
    if file:
        if os.path.isfile(file.path):
            os.remove(file.path)

    if hasattr(sender, 'file'):
        if sender.data:
            if os.path.isfile(sender.data.path):
                os.remove(sender.data.path)
