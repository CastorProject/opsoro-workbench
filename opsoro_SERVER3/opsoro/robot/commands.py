from django.db.models import Q

from documents.models import File, Sound, SoundCollection

from .models import Expression, Robot

# from play.plays import update_user_robot

try:
    import simplejson as json
except ImportError:
    import json

# Virtual model:
#   dofs, sound, refresh

# Robot:
#   stop
#   dof:
#       index, tags
#   sound:
#       filename, text
#   emotion:
#       poly, emoji, shortcode, name, index


def command_robot(user, kwargs):
    try:
        kwargs = json.loads(kwargs)
    except ValueError:
        print('Invalid JSON')
        return None

    print('set', user, kwargs)
    if 'action' in kwargs:
        action = kwargs['action']
        getset = action.split('_')[0]

        if action in functions:
            try:
                robot = Robot.objects.filter(owner=user)[0]
            except Exception as e:
                print('No robot for user %s' % user.username)
                return {'error': 'No robot available.'}

            response = functions[action](robot, kwargs)
            if response is not None:
                if getset == 'get':
                    response.update({'action': 'data'})
                    return response
                elif getset == 'set':
                    response.update({'action': 'robot'})
                    return response

    return json.dumps({'error': 'Invalid args.'})


def stop(robot, kwargs):
    return {'stop': True}


def set_sound(robot, kwargs):
    sound = None
    text = None
    file_data = None

    if 'sound' in kwargs:
        sound = kwargs['sound']
    if 'text' in kwargs:
        text = kwargs['text']
    if 'file' in kwargs:
        file_data = kwargs['file']

    msg = ''

    if sound is None:
        return

    if sound == 'tts':
        if text is None:
            return
        msg = text
    else:
        if file_data is None:
            return

        file_name = file_data
        file_url = None
        if type(file_data) is dict:
            if 'name' in file_data:
                file_name = file_data['name']
            if 'url' in file_data:
                file_url = file_data['url']

        try:
            sound_collections = SoundCollection.objects.filter(Q(public=True) | Q(author=robot.owner))
            for collection in sound_collections.all():
                try:
                    snds = collection.sounds.filter(name__iexact=file_name)
                    for snd in snds:
                        if len(snds) == 1 or file_url is None or str(snd.data.url).lower() == str(file_url).lower():
                            msg = {'name': snd.name, 'url': snd.data.url}
                except Exception as e:
                    pass

        except Exception as e:
            print(e)
            return

    return {'sound': sound, 'msg': msg}


def get_sounds(robot, kwargs):
    # Get public and personal sounds
    sound_collections = SoundCollection.objects.filter(Q(public=True) | Q(author=robot.owner))

    if len(sound_collections) == 0:
        return

    return_sounds = {}

    for collection in sound_collections.all().order_by('name'):
        return_sounds[collection.name] = []
        for sound in collection.sounds.all().order_by('name'):
            snd = {'name': sound.name, 'url': sound.data.url}
            return_sounds[collection.name].append(snd)

    return {'sounds': return_sounds}


def set_dofs(robot, kwargs):
    # if type(value) is list:
    #     pass
    dofs = None
    if 'dofs' in kwargs:
        dofs = kwargs['dofs']
    if dofs is None:
        return

    # robot_dofs = robot.dofs.all()
    return_dofs = []
    for dof in dofs:
        return_dofs.append({'tags': dof['name'].split(' '), 'value': dof['value']})

    return {'dofs': return_dofs}

# def set_dof(robot, **kwargs):
#     print('dof', robot, kwargs)
#     pass


def set_expression(robot, kwargs):
    poly_index = None
    name = None
    intensity = None
    expression = None
    symbol = None

    if 'poly_index' in kwargs:
        poly_index = kwargs['poly_index']
    if 'intensity' in kwargs:
        intensity = kwargs['intensity']
    if 'name' in kwargs:
        name = kwargs['name']
    if 'symbol' in kwargs:
        symbol = kwargs['symbol']

    if intensity is None:
        intensity = 1

    if poly_index is not None:
        expression = Expression.objects.filter(robot=robot, poly_index=poly_index)
        print("EXPRESSION:", expression)

    if name is not None:
        expression = Expression.objects.filter(robot=robot, name__iexact=name)

    if symbol is not None:
        expression = Expression.objects.filter(robot=robot, symbol_code__iexact=symbol)

    if expression is None or len(expression) < 1:
        return
    expression = expression[0]

    robot_dofs = []
    return_dofs = []

    for dof in expression.dofs.all():
        dof_tmp = dof
        dof_tmp.value *= intensity
        return_dofs.append({'tags': dof_tmp.tags.split(' '), 'value': dof_tmp.value})
        robot_dofs.append(dof_tmp)

    robot.dofs.set(robot_dofs)
    robot.save()

    return {'dofs': return_dofs}


def get_expressions(robot, kwargs):
    expressions = Expression.objects.filter(robot=robot)

    if expressions is None or len(expressions) < 1:
        return

    return_expressions = []

    for exp in expressions:
        tmp_exp = {
            'name':         exp.name,
            'symbol_code':  exp.symbol_code,
            'short_code':   exp.short_code,
            'poly_index':   exp.poly_index,
        }
        return_expressions.append(tmp_exp)

    return {'expressions': return_expressions}


def get_dofs(robot, kwargs):
    return_dofs = []

    for modlink in robot.modulelink.all():
        mod = modlink.module
        for dof in mod.dof.all():
            tmp_dof = {}
            tmp_dof['name'] = str(mod.name + ' ' + modlink.name + ' ' + dof.name).replace('  ', ' ')
            return_dofs.append(tmp_dof)

    return {'dofs': return_dofs}


def get_dofs_values(robot, kwargs):
    return_dofs = []

    for dof in robot.dofs.all():
        return_dofs.append({'tags': dof.tags.split(' '), 'value': dof.value})

    return {'dofs': return_dofs}


functions = {
    'set_stop':         stop,
    'set_sound':        set_sound,
    'get_sounds':       get_sounds,
    'set_dofs':         set_dofs,
    'get_dofs':         get_dofs,
    'get_dofs_values':  get_dofs_values,
    'set_expression':   set_expression,
    'get_expressions':  get_expressions,
}
