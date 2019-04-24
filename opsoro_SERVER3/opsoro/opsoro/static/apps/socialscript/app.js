
ko.bindingHandlers.outputswitch = {
  init: function(element, valueAccessor) {
    $(element).change(function() {
      model.fileIsModified(true);
      var value = valueAccessor();
      if ($(element).prop("checked")) {
        value("wav");
      } else {
        value("tts");
      }
    });
  },
  update: function(element, valueAccessor, allBindings) {
    var value = valueAccessor();
    var valueUnwrapped = ko.unwrap(value);
    $(element).prop("checked", valueUnwrapped == "tts" ? false : true);
  }
};

var switchID = 0; // Variable to generate unique IDs for toggle switches

// Here's my data model
var VoiceLine = function(emotion, output, tts, wav) {
  var self = this;
	if (emotion == undefined) {
		return
	}
  self.emotion = ko.observable(emotion);

  self.output = ko.observable(output || "tts");
  self.tts = ko.observable(tts || "");
  self.wav = ko.observable(wav || "");

  self.isPlaying = ko.observable(false);
  self.hasPlayed = ko.observable(false);

  self.switchID = "output-switch-" + switchID++;

  self.toggleOutput = function() {
    model.fileIsModified(true);
    if (this.output() == "tts") {
      this.output("wav");
    } else {
      this.output("tts");
    }
  };

  self.contentPreview = ko.pureComputed(function() {
    if (self.output() == "tts") {
      // Generate tts preview html
      return "<span class='fa fa-comment'></span> " + self.tts();
    } else {
      // Generate wav preview html
      return "<span class='fa fa-music'></span> " + self.wav().name;
    }
  });

  self.emoji = ko.pureComputed(function() {
    return self.emotion().symbol_code;
  });

  self.modified = function() {
    model.fileIsModified(true);
  }

  self.pressPlay = function() {
    if (self.isPlaying()) {
      robotSendStop();
      self.isPlaying(false);
      self.hasPlayed(true);
    } else {
      if (model.selectedVoiceLine() != undefined) {
        model.selectedVoiceLine().isPlaying(false);
      }
      model.selectedVoiceLine(self);
      // if (self.emotion().poly_index) {
      // robotSendEmotion(self.emotion().poly_index, -1);
      robotSendEmotionSymbol(self.emotion().symbol_code, -1);
      // }
      if (this.output() == "tts") {
        robotSendTTS(self.tts());
      } else {
        robotSendSound(self.wav());
      }
      self.isPlaying(true);

      setTimeout(function () {
        self.isPlaying(false);
        self.hasPlayed(true);
      }, 2000);


    }
  };

  self.pickEmotion = function() {
    if (model.fileIsLocked()) {
      return;
    }

    model.selectedVoiceLine(self);
    $("#PickEmotionModal").foundation("open");
  };
};

var AppModel = function() {
  var self = this;

  self.fileIsLocked = ko.observable(false);
  self.fileIsModified = ko.observable(false);
  self.fileStatus = ko.observable("");
  self.fileExtension = ko.observable(".soc");

  self.sounds = ko.observableArray(); //sounds_data;
  self.emotions = ko.observableArray(); //emotions_data;

  self.selectedVoiceLine = ko.observable();

  self.voiceLines = ko.observableArray();
  self.fixedVoiceLine = ko.observable();
  self.fixedAvatars = ko.observableArray();

  self.newFileData = function() {
    self.voiceLines.removeAll();
    self.voiceLines.push(new VoiceLine(self.emotions()[0], "tts", "", ""));
    self.unlockFile();
    self.fileIsModified(false);
  };

  self.toggleLocked = function() {
    if (self.fileIsLocked()) {
      self.unlockFile();
    } else {
      self.lockFile();
    }
  };
  self.lockFile = function() {
    self.fileIsLocked(true);
    self.fileStatus("Locked")
  };
  self.unlockFile = function() {
    self.fileIsLocked(false);
    self.fileStatus("Editing")
  };

  self.addLine = function() {
    self.fileIsModified(true);
    self.voiceLines.push(new VoiceLine(self.emotions()[0], "tts", "", ""));
    window.scrollTo(0, document.body.scrollHeight);
  };

  self.removeLine = function(line) {
    self.fileIsModified(true);
    self.voiceLines.remove(line);
  };

  self.loadFileData = function(data) {
    if (data == undefined) {
      return;
    }
    // Load script
    self.voiceLines.removeAll();

    var dataobj = JSON.parse(data);

    $.each(dataobj.voice_lines, function(idx, line) {
      var emo = self.emotions()[0];
      $.each(self.emotions(), function(idx, emot) {
        if (emot.name.toLowerCase() == line.emotion.toLowerCase()) {
          emo = emot;
        }
      });
      if (line.output.type == "tts") {
        self.voiceLines.push(new VoiceLine(emo, line.output.type, line.output.data, ""));
      } else {
        var snd = self.sounds()[0];
        $.each(self.sounds(), function(idx, sound) {
          if (line.output.data.name != undefined && sound.name.toLowerCase() == line.output.data.name.toLowerCase()) {
            snd = sound;
          }
        });
        self.voiceLines.push(new VoiceLine(emo, line.output.type, "", snd));
      }
    });
    self.fileIsModified(false);
    self.lockFile();
    return true;
  };

  self.saveFileData = function() {
    var file_data = {
      voice_lines: []
    };
    $.each(self.voiceLines(), function(idx, item) {
      var line = {};
      line.emotion = item.emotion().name;
      line.output = {};
      if (item.output() == "tts") {
        line.output.type = "tts";
        line.output.data = item.tts();
      } else if (item.output() == "wav") {
        line.output.type = "wav";
        line.output.data = item.wav();
      } else {
        line.output.type = "tts";
        line.output.data = "";
      }
      file_data.voice_lines.push(line);
    });
    self.fileIsModified(false);
    return ko.toJSON(file_data, null, 2);
  };

  self.changeEmotion = function(emotion) {
    self.fileIsModified(true);
    self.selectedVoiceLine().emotion(emotion);
    $("#PickEmotionModal").foundation("close");
  };

  self.changeFixedEmotion = function(emotion) {
    self.fixedVoiceLine.emotion(emotion);
  };
	self.init = function() {
		robotGetSounds();
		robotGetExpressions();
	};

	self.sounds_loaded = function(sounds) {
		self.sounds.removeAll();
		for (var collection in sounds) {
			if (collection != undefined) {
        $.each(sounds[collection], function(idx, snd) {
          self.sounds.push(snd);
        });
			}
		}
    self.all_loaded();
	};
	self.expressions_loaded = function(expressions) {
    self.emotions.removeAll();
		$.each(expressions, function(idx, exp) {
      self.emotions.push(exp);
			// self.fixedAvatars.push(new VoiceLine(exp, "tts", "", ""));
		});
		// self.fixedVoiceLine = new VoiceLine(self.emotions()[0], "tts", "", "");

    self.all_loaded();
	};
  self.all_loaded = function() {
    if (self.sounds().length == 0 || self.emotions().length == 0) {
      return;
    }
    config_file_operations("", self.fileExtension(), self.saveFileData, self.loadFileData, self.newFileData);
  }
};
var model;

$(document).ready(function() {
  // This makes Knockout get to work
  model = new AppModel();
  ko.applyBindings(model);
  model.fileIsModified(false);

	// model.init();
  // setTimeout(model.init, 500);

});
