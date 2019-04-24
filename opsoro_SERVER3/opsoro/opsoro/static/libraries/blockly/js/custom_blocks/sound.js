Blockly.Lua.addReservedWords("Sound");

Blockly.Blocks['sound_saytts'] = {
  init: function() {
    this.appendDummyInput()
        .appendField(new Blockly.FieldImage("/static/libraries/font-awesome/svg/white/volume-down.svg", 16, 18, ""))
        .appendField("Say")
        .appendField(new Blockly.FieldTextInput("I am a robot!"), "TEXT");
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.setColour(300);
    this.setTooltip('Say something using the text-to-speech function.');
  }
};
Blockly.Lua['sound_saytts'] = function(block) {
  var text_text = block.getFieldValue('TEXT');
  var code = 'Sound:say_tts("' + text_text + '")\n';
  return code;
};
Blockly.JavaScript['sound_saytts'] = function(block) {
  var text_text = block.getFieldValue('TEXT');
  // var code = 'robotSendTTS("' + text_text + '");\n';
  var code = 'sendCommand(\'{"action": "set_sound", "sound": "tts", "text": "' + text_text + '"}\');\n';
  return code;
};
Blockly.Blocks['sound_play'] = {
  init: function() {
    this.appendDummyInput()
        .appendField(new Blockly.FieldImage("/static/libraries/font-awesome/svg/white/volume-down.svg", 16, 18, ""))
        .appendField("Play")
        .appendField(new Blockly.FieldDropdown(model.blockly_sounds), "FILENAME");
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.setColour(300);
    this.setTooltip('Play a sound sample.');
  }
};
Blockly.Lua['sound_play'] = function(block) {
  var dropdown_filename = block.getFieldValue('FILENAME');
  var code = 'Sound:play_file("' + dropdown_filename + '")\n';
  return code;
};
Blockly.JavaScript['sound_play'] = function(block) {
  var dropdown_filename = block.getFieldValue('FILENAME');
  // var code = 'robotSendSound("' + dropdown_filename + '");\n';
  var code = 'sendCommand(\'{"action": "set_sound", "sound": "file", "file": "' + dropdown_filename + '"}\');\n';
  return code;
};
