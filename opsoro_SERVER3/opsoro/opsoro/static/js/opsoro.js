function showMainError(msg) {
  $('#errors').append("<div class='callout alert' style='display: none;' data-closable>" + msg + "<button class='close-button' aria-label='Dismiss' type='button' data-close><span aria-hidden='true'>&times;</span></button></div>");
  $('#errors .callout.alert').slideDown("fast", function() {});
  setTimeout(function () {
    $('#errors .callout.alert').slideUp( "slow", function() {
      $('#errors .callout.alert').remove();
    });
  }, 5000);
}
function showMainWarning(msg) {
  $('#errors').append("<div class='callout warning' style='display: none;' data-closable>" + msg + "<button class='close-button' aria-label='Dismiss' type='button' data-close><span aria-hidden='true'>&times;</span></button></div>");
  $('#errors .callout.warning').slideDown("fast", function() {});
  setTimeout(function () {
    $('#errors .callout.warning').slideUp( "slow", function() {
      $('#errors .callout.warning').remove();
    });
  }, 4000);
}
function showMainMessage(msg) {
  $('#errors').append("<div class='callout primary' style='display: none;' data-closable>" + msg + "<button class='close-button' aria-label='Dismiss' type='button' data-close><span aria-hidden='true'>&times;</span></button></div>");
  $('#errors .callout.primary').slideDown("fast", function() {});
  setTimeout(function () {
    $('#errors .callout.primary').slideUp( "slow", function() {
      $('#errors .callout.primary').remove();
    });
  }, 3000);
}
function showMainSuccess(msg) {
  $('#errors').append("<div class='callout success' style='display: none;' data-closable>" + msg + "<button class='close-button' aria-label='Dismiss' type='button' data-close><span aria-hidden='true'>&times;</span></button></div>");
  $('#errors .callout.success').slideDown("fast", function() {});
  setTimeout(function () {
    $('#errors .callout.success').slideUp( "slow", function() {
      $('#errors .callout.success').remove();
    });
  }, 2000);
}
var popup_classes;
function showPopup(sIcon, sTitle, sClass, sContent) {
  popup_classes = sClass;
  $('#popup').addClass(popup_classes);
  $('#popup .titlebar .titleicon span').addClass(sIcon);
  $('#popup .titlebar .title').html(sTitle);

  if (sContent != '') {
    $('#popup .content').html(sContent);
  }

  $('#popup .btnClose').off('click');
  $('#popup .btnClose').on('click', closePopup);

  // Open message popup
  $('#popup').foundation('open');

}

function closePopup() {
  $('#popup').foundation('close');

  // Clear text & icon
  $('#popup .titlebar .titleicon span').removeClass();
  $('#popup .titlebar .titleicon span').addClass('fa');
  $('#popup .titlebar .title').html('');
  $('#popup .content').html('');

  $('#popup').removeClass(popup_classes);
  $('#popup').addClass('reveal');
}

function showMessagePopup(sIcon, sTitle, sText, handlers) {
  $('#message_popup .titlebar .titleicon span').addClass(sIcon);
  $('#message_popup .titlebar .title').html(sTitle);
  $('#message_popup .content .text').html(sText);

  data = {};

  // Input enabling
  if (handlers.inputText != undefined) {
    $('#message_popup .inputText').removeClass('hide');
    $('#message_popup .inputText').on('input', handlers.inputText);
  }

  // Button click handlers
  if (handlers.btnOk != undefined) {
    $('#message_popup .btnOk').removeClass('hide');
    $('#message_popup .btnOk').on('click', handlers.btnOk);
  }
  if (handlers.btnYes != undefined) {
    $('#message_popup .btnYes').removeClass('hide');
    $('#message_popup .btnYes').on('click', handlers.btnYes);
  }
  if (handlers.btnNo != undefined) {
    $('#message_popup .btnNo').removeClass('hide');
    $('#message_popup .btnNo').on('click', handlers.btnNo);
  }
  if (handlers.btnSave != undefined) {
    $('#message_popup .btnSave').removeClass('hide');
    $('#message_popup .btnSave').on('click', handlers.btnSave);
  }
  if (handlers.btnCancel != undefined) {
    $('#message_popup .btnCancel').removeClass('hide');
    $('#message_popup .btnCancel').on('click', handlers.btnCancel);
  }

  $('#message_popup .btnClose').off('click');
  $('#message_popup .btnClose').on('click', closeMessagePopup);

  // Open message popup
  if (!$('#message_popup').hasClass('open')) {
    $('#message_popup').foundation('open');
  }
}

function closeMessagePopup() {
  $('#message_popup').foundation('close');

  // Clear text & icon
  $('#message_popup .titlebar .titleicon span').removeClass();
  $('#message_popup .titlebar .titleicon span').addClass('fa');
  $('#message_popup .titlebar .title').html('');
  $('#message_popup .content .text').html('');

  // Clear inputs
  $('#message_popup .inputText').val('');

  // Hide inputs
  $('#message_popup .inputText').addClass('hide');

  // Hide buttons
  $('#message_popup .btnOk').addClass('hide');
  $('#message_popup .btnYes').addClass('hide');
  $('#message_popup .btnNo').addClass('hide');
  $('#message_popup .btnSave').addClass('hide');
  $('#message_popup .btnCancel').addClass('hide');

  // Remove click handlers
  $('#message_popup .btnOk').off('click');
  $('#message_popup .btnYes').off('click');
  $('#message_popup .btnNo').off('click');
  $('#message_popup .btnSave').off('click');
  $('#message_popup .btnCancel').off('click');

  $('#message_popup .btnClose').off('click');
}

function popupWindow(mylink, windowname) {
  if (!window.focus) return true;
  var href;
  if (typeof(mylink) == 'string')
    href = mylink;
  else
    href = mylink.href;
  window.open(href, windowname, 'width=400,height=500,scrollbars=no');
  return false;
}

function showAppContent() {
  $('.app-content-placeholder').hide();
  $('.app-content').show();
}

// -------------------------------------------------------------------------------------------------------
// Socket connection
//--------------------------------------------------------------------------------------------------------
// Setup websocket connection.
var app_socket_handler = undefined;
var socket = null;
var connReady = false;

function connectSocket() {
  var conn_prefix = 'wss';
  var conn_suffix = '';
  if (location.protocol != 'https:') { conn_prefix = 'ws'; }

  // Robot socket
  if (typeof virtual_robot != 'undefined' && virtual_robot == true) { conn_suffix = 'robot/'; }

  socket = new WebSocket(conn_prefix + '://' + window.location.host + '/ws/' + conn_suffix);

  socket.onopen = function(){
    console.log('SockJS connected.');
    connReady = true;
    console.log('SockJS authenticated.');
    //
    // if (typeof virtual_robot != 'undefined' && virtual_robot) {
    //  socket.send(JSON.stringify({action: 'robot'}));
    // }
    if (typeof model != 'undefined' && typeof model.init === 'function') {
      model.init();
    }
  };

  console.log("init onmessage");
  socket.onmessage = function(e) {
    try {
      let msg = JSON.parse(e.data);
      console.log("message: " + msg);
      console.log("action: " + msg.action);
      switch(msg.action) {
        case 'refresh':
          setTimeout(function() { location.reload(); }, 1000);
          break;
        case 'info':
          if (typeof msg.type != 'undefined' && typeof msg.text != 'undefined') {
            switch(msg.type) {
              case 'popup':
                showMessagePopup('fa-info', 'Server info', msg.text, {btnOk: function() { location.reload(); }});
                break;
              case 'error':
                showMainError(msg.text);
                break;
              case 'warning':
                showMainWarning(msg.text);
                break;
              case 'message':
                showMainMessage(msg.text);
                break;
              case 'success':
                showMainSuccess(msg.text);
                break;
            }
          }
          break;
        case 'users':
          var text = ' user';
          if (msg.count > 1) { text += 's' }
          text += ' connected.';

          $('.online_users').html(msg.count + text);
          break;
        case 'apps':
          $('.app-active').css('display', 'none');
          $('.app-locked').css('display', 'none');

          if (typeof msg.active != 'undefined') {
            for (let i = 0; i < msg.active.length; i++) {
              let app = msg.active[i];
              $('.' + app + ' .app-active').css('display', 'inline-block');
            }
          }
          if (typeof msg.locked != 'undefined') {
            for (let i = 0; i < msg.locked.length; i++) {
              let app = msg.locked[i];
              $('.' + app + ' .app-locked').css('display', 'inline-block');
            }
          }
          break;
        case 'app':
          console.log(msg.data);
          if (app_socket_handler != undefined) {
            app_socket_handler(msg.data);
          }
          break;
        case 'robot':
          console.log(virtualModel);
          console.log(msg.dofs);
          if (typeof virtualModel != 'undefined') {
            if (typeof msg.dofs != 'undefined' && typeof virtualModel.update_dofs === 'function') {
              virtualModel.update_dofs(msg.dofs);
            }
            if (typeof msg.sound != 'undefined' && typeof msg.msg != 'undefined' && typeof virtualModel.update_sound === 'function') {
              virtualModel.update_sound(msg.sound, msg.msg);
            }
            if (typeof msg.stop != 'undefined' && typeof virtualModel.stop_all === 'function') {
              virtualModel.stop_all();
            }
            if (typeof msg.refresh != 'undefined') {
              location.reload();
            }

            if (typeof msg.dofs != 'undefined') {
              // location.reload();
            }
          }

          break;
        case 'data':
          if (typeof model != 'undefined') {
            if (typeof msg.sounds != 'undefined' && typeof model.sounds_loaded === 'function') {
              model.sounds_loaded(msg.sounds);
            }
            if (typeof msg.expressions != 'undefined' && typeof model.expressions_loaded === 'function') {
              model.expressions_loaded(msg.expressions);
            }
            if (typeof msg.dofs != 'undefined' && typeof model.dofs_loaded === 'function') {
              model.dofs_loaded(msg.dofs);
            }
          }

          break;
        // case 'shutdown':
        //   showMessagePopup('fa-info', 'Server info', msg.text, {btnOk: function() { location.reload(); }});
        //   break;
      }
    } catch (e) {
      console.log(e);
    } finally {
      let x;
    }

  };

  socket.onclose = function() {
    console.log('SOCKET close');
    socket = null;

    // Only reconnect if the connection was successfull in the first place
    if (connReady) {
      setTimeout(function() {
        var retry_socket = setInterval(function () {
          connectSocket();
          setTimeout(function() {
            if (connReady) {
              clearInterval(retry_socket);
              if (typeof virtual_robot != 'undefined' && virtual_robot) {
                location.reload();
              } else {
                location.reload();
              }
            }
          }, 500);
        }, 1000);

        // showMainError('Disconnected from robot, trying to reconnect...');
        $('.online_users').html('Disconnected, trying to reconnect...');
        $('.active_apps').html('');
        if (typeof virtual_robot != 'undefined' && virtual_robot) {
          let x;
        } else {
          setTimeout(function() { location.reload(); }, 5000);
        }
      }, 500);
    }

    connReady = false;
  };
}
$(document).ready(function() {
  connectSocket();

  //Kleur navbar veranderen wnr we scrollen
  $(document).scroll(function() {
    var scroll = $(this).scrollTop();
    var start_fade_pos = 50;
    var end_fade_pos = 150;
    if (scroll > start_fade_pos) {
      $('#header_main').removeClass('on-top');
      var opacity = (scroll - start_fade_pos) / (end_fade_pos - start_fade_pos);
      if (opacity > 1.0) { opacity = 1.0; }
      if (opacity < 0.0) { opacity = 0.0; }
      $('#header_main').css('background-color', 'rgba(39, 41, 42, ' + opacity + ')');
    } else {
      $('#header_main').css('background-color', 'rgba(39, 41, 42, 0.0)');
      $('#header_main').addClass('on-top');
    }
  });

  $(document).foundation();
});
// -------------------------------------------------------------------------------------------------------
// Robot control functions
// -------------------------------------------------------------------------------------------------------
function sendCommand(args) {
  try {
    if (connReady) {
      if (typeof args === 'string') {
        args = JSON.parse(args);
      }
      socket.send(JSON.stringify(args));
      return true;
    } else {
      $.ajax({
        dataType: 'json',
        type: 'POST',
        url: '/robot/command/',
        data: args,
        success: function(data) {
          if (!data.success) {
            showMainError(data.message);
          }
        }
      });
      // return true;
    }
  } catch (e) {
    console.log(e);
  }
  return false;
}

function robotSendReceiveConfig(config_data) {
  var json_data = ko.toJSON(config_data, null, 2);
  $.ajax({
    dataType: 'json',
    type: 'POST',
    url: '/robot/config/',
    data: {
      config_data: json_data
    },
    success: function(data) {
      if (!data.success) {
        showMainError(data.message);
      } else {
        return data.config;
      }
    }
  });
}

function robotSendEmotionRPhi(r, phi, time) {
  // time: -1 for default smooth transition time
  // Constrain phi
  phi = Math.max(Math.min(phi, 360), 0);
  poly_index = Math.floor(phi / 18);

  msg_data = {
    'action': 'set_expression',
    'poly_index': poly_index,
    'intensity': r,
    'time': time,
  };
  // Try sending thru sockets
  if (sendCommand(msg_data)) {
    return;
  }
  // If there is no socket connection, send thru ajax
  $.ajax({
    dataType: 'json',
    type: 'POST',
    url: '/robot/emotion/',
    data: msg_data,
    success: function(data) {
      if (!data.success) {
        showMainError(data.message);
      }
    }
  });
}

function robotSendEmotionPoly(poly_index, time) {
  // time: -1 for default smooth transition time

  msg_data = {
    'action': 'set_expression',
    'poly_index': poly_index,
    'time': time,
  };
  // Try sending thru sockets
  if (sendCommand(msg_data)) {
    return;
  }
  // If there is no socket connection, send thru ajax
  $.ajax({
    dataType: 'json',
    type: 'POST',
    url: '/robot/emotion/',
    data: msg_data,
    success: function(data) {
      if (!data.success) {
        showMainError(data.message);
      }
    }
  });
}

function robotSendEmotionSymbol(symbol, time) {
  // time: -1 for default smooth transition time

  msg_data = {
    'action': 'set_expression',
    'symbol': symbol,
    'time': time,
  };
  // Try sending thru sockets
  if (sendCommand(msg_data)) {
    return;
  }
  // If there is no socket connection, send thru ajax
  $.ajax({
    dataType: 'json',
    type: 'POST',
    url: '/robot/emotion/',
    data: msg_data,
    success: function(data) {
      if (!data.success) {
        showMainError(data.message);
      }
    }
  });
}

function robotSendEmotionName(name, time) {
  // time: -1 for default smooth transition time
  msg_data = {
    'action': 'set_expression',
    'name': name,
    'time': time,
  };
  // Try sending thru sockets
  if (sendCommand(msg_data)) {
    return;
  }
  // If there is no socket connection, send thru ajax
  $.ajax({
    dataType: 'json',
    type: 'POST',
    url: '/robot/emotion/',
    data: msg_data,
    success: function(data) {
      if (!data.success) {
        showMainError(data.message);
      }
    }
  });
}

function robotSendDOF(dofs) {
  msg_data = {
    'action': 'set_dofs',
    'dofs': dofs,
  };
  // Try sending thru sockets
  if (sendCommand(msg_data)) {
    return;
  }
  // If there is no socket connection, send thru ajax
  $.ajax({
    dataType: 'json',
    type: 'POST',
    url: '/robot/dof/',
    data: msg_data,
    success: function(data) {
      if (!data.success) {
        showMainError(data.message);
      }
    }
  });
}

function robotGetSounds() {
  msg_data = {
    'action': 'get_sounds',
  };
  // Try sending thru sockets
  if (sendCommand(msg_data)) {
    return;
  }
  // If there is no socket connection, send thru ajax
  $.ajax({
    dataType: 'json',
    type: 'GET',
    url: '/robot/sounds/',
    data: msg_data,
    success: function(data) {
      if (!data.success) {
        showMainError(data.message);
      } else {
        return data;
      }
    }
  });
}

function robotGetExpressions() {
  msg_data = {
    'action': 'get_expressions',
  };
  // Try sending thru sockets
  if (sendCommand(msg_data)) {
    return;
  }
  // If there is no socket connection, send thru ajax
  $.ajax({
    dataType: 'json',
    type: 'GET',
    url: '/robot/expressions/',
    data: msg_data,
    success: function(data) {
      if (!data.success) {
        showMainError(data.message);
      } else {
        return data;
      }
    }
  });
}

function robotGetDofs() {
  msg_data = {
    'action': 'get_dofs',
  };
  // Try sending thru sockets
  if (sendCommand(msg_data)) {
    return;
  }
  // If there is no socket connection, send thru ajax
  $.ajax({
    dataType: 'json',
    type: 'GET',
    url: '/robot/dofs/',
    data: msg_data,
    success: function(data) {
      if (!data.success) {
        showMainError(data.message);
      } else {
        return data;
      }
    }
  });
}

function robotSendReceiveAllDOF(dofData) {
  if (dofData == undefined) {
    $.ajax({
      dataType: 'json',
      type: 'POST',
      url: '/robot/dofs/',
      success: function(data) {
        if (!data.success) {
          showMainError(data.message);
        } else {
          return data.dofs;
        }
      }
    });
    return;
  }
  var json_data = ko.toJSON(dofData, null, 2);
  $.ajax({
    dataType: 'json',
    type: 'POST',
    url: '/robot/dofs/',
    data: {
      dofdata: json_data
    },
    success: function(data) {
      if (!data.success) {
        showMainError(data.message);
      } else {
        return data.dofs;
      }
    }
  });
}

function robotSendServo(pin, value) {
  // $.ajax({
  //   dataType: 'json',
  //   type: 'POST',
  //   url: '/robot/servo/',
  //   data: {'pin_number': pin, 'value': value},
  //   success: function(data){
  //     if (!data.success) {
  //       showMainError(data.message);
  //     }
  //   }
  // });
}

function robotSendTTS(text) {
  msg_data = {
    'action': 'set_sound',
    'sound': 'tts',
    'text': text,
  };
  // Try sending thru sockets
  if (sendCommand(msg_data)) {
    return;
  }
  // If there is no socket connection, send thru ajax
  $.ajax({
    dataType: 'json',
    type: 'GET',
    url: '/robot/tts/',
    data: msg_data,
    success: function(data) {
      if (!data.success) {
        showMainError(data.message);
      }
    }
  });
}

function robotSendSound(soundName) {
  msg_data = {
    'action': 'set_sound',
    'sound': 'file',
    'file': soundName,
  };
  // Try sending thru sockets
  if (sendCommand(msg_data)) {
    return;
  }
  // If there is no socket connection, send thru ajax
  $.ajax({
    dataType: 'json',
    type: 'GET',
    url: '/robot/sound/',
    data: msg_data,
    success: function(data) {
      if (data.status == 'error') {
        showMainError(data.message);
      }

    }
  });
}

function robotSendStop() {
  msg_data = {
    'action': 'set_stop',
  };
  // Try sending thru sockets
  if (sendCommand(msg_data)) {
    return;
  }
  // If there is no socket connection, send thru ajax
  $.ajax({
    dataType: 'json',
    type: 'GET',
    url: '/robot/stop/',
    success: function(data) {
      if (data.status == 'error') {
        showMainError(data.message);
      }

    }
  });
}



$('a[href*="#"]:not([href="#"])').click(function() {
  if (location.pathname.replace(/^\//, '') == this.pathname.replace(/^\//, '') ||
    location.hostname == this.hostname) {

    var target = $(this.hash);
    target = target.length ? target : $('[name=' + this.hash.slice(1) + ']');
    if (target.length) {
      $('html,body').stop();
      $('html,body').animate({
        scrollTop: target.offset().top - $('#header_main').height()
      }, 600);
      return false;
    }
  }
});

(function(i, s, o, g, r, a, m) {
  i['GoogleAnalyticsObject'] = r;
  i[r] = i[r] || function() {
    (i[r].q = i[r].q || []).push(arguments)
  }, i[r].l = 1 * new Date();
  a = s.createElement(o),
    m = s.getElementsByTagName(o)[0];
  a.async = 1;
  a.src = g;
  m.parentNode.insertBefore(a, m)
})(window, document, 'script', 'https://www.google-analytics.com/analytics.js', 'ga');

ga('create', 'UA-85675286-1', 'auto');
ga('send', 'pageview');

window.onload = function() {
  // var placeholders = $('.placeholder');

  $('.placeholder').each(function(i) {
    var placeholder = $(this)[0];
    var small = $(this).find('.img-small')[0]; // This is your rel value

    // 1: load small image and show it
    var imgSmall = new Image();
    imgSmall.src = small.src;
    imgSmall.onload = function () {
     small.classList.add('loaded');
    };

    // 2: load large image
    var imgLarge = new Image();
    imgLarge.src = placeholder.dataset.large;
    // imgLarge.alt = placeholder.dataset.alt;
    imgLarge.onload = function () {
      imgLarge.classList.add('loaded');
      placeholder.classList.add('big-loaded');
    };
    placeholder.appendChild(imgLarge);

  });
};


function ClickToEdit(value, placeholder) {
  var self = this;
  // Data
  self.value = ko.observable(value);
  self.editing = ko.observable(false);
  self.placeholder = placeholder || "empty";

  // Behaviors
  self.edit = function () {
    self.editing(true)
  };

  self.displayValue = ko.pureComputed(function() {
      if(self.value() == "") {
            return "<i style='color: #888;'>" + self.placeholder +"</i>"
      }
      return self.value();
  }, self);

}

function Say(text) {
  var u = new SpeechSynthesisUtterance();
  u.text = text;
  u.lang = 'en-US';
  u.rate = 1.2;
  // u.onend = function(event) { alert('Finished in ' + event.elapsedTime + ' seconds.'); }
  speechSynthesis.speak(u);
}
