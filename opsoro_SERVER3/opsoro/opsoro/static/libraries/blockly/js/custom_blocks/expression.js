Blockly.Lua.addReservedWords("Expression");

Blockly.Blocks['expression_setekman'] = {
  init: function() {
    this.appendDummyInput()
        .appendField(new Blockly.FieldImage("/static/libraries/font-awesome/svg/white/smile-o.svg", 16, 18, ""))
        .appendField("set emotion to")
        .appendField(new Blockly.FieldDropdown(model.blockly_expressions), "EMOTION");
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.setColour(105);
    this.setTooltip('Set the current facial expression using Ekman\'s basic emotions');
  }
};
Blockly.Lua['expression_setekman'] = function(block) {
  var dropdown_emotion = block.getFieldValue('EMOTION');
  var code = 'Expression:set_emotion_name("' + dropdown_emotion + '")\n';
  return code;
};
Blockly.JavaScript['expression_setekman'] = function(block) {
  var dropdown_emotion = block.getFieldValue('EMOTION');
  // var code = 'robotSendEmotionName("' + dropdown_emotion +'", -1);\n';
  var code = 'sendCommand(\'{"action": "set_expression", "name": "' + dropdown_emotion + '", "time": -1}\');\n';
  return code;
};

Blockly.Blocks['expression_setva'] = {
  init: function() {
    this.appendDummyInput()
        .appendField(new Blockly.FieldImage("/static/libraries/font-awesome/svg/white/smile-o.svg", 16, 18, ""))
        .appendField("Set emotion to");
    this.appendValueInput("VALENCE")
        .setCheck("Number")
        .setAlign(Blockly.ALIGN_RIGHT)
        .appendField("Valence");
    this.appendValueInput("AROUSAL")
        .setCheck("Number")
        .setAlign(Blockly.ALIGN_RIGHT)
        .appendField("Arousal");
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.setColour(105);
    this.setTooltip('Set the current facial expression using Valence and Arousal. Parameters range from -1.0 to +1.0.');
  }
};
Blockly.Lua['expression_setva'] = function(block) {
  var value_valence = Blockly.Lua.valueToCode(block, 'VALENCE', Blockly.Lua.ORDER_ATOMIC);
  var value_arousal = Blockly.Lua.valueToCode(block, 'AROUSAL', Blockly.Lua.ORDER_ATOMIC);

  var code = 'Expression:set_emotion_val_ar(' + value_valence + ', ' + value_arousal +')\n';
  return code;
};
Blockly.JavaScript['expression_setva'] = function(block) {
  var value_valence = Blockly.JavaScript.valueToCode(block, 'VALENCE', Blockly.JavaScript.ORDER_ATOMIC);
  var value_arousal = Blockly.JavaScript.valueToCode(block, 'AROUSAL', Blockly.JavaScript.ORDER_ATOMIC);
  // var code = 'robotSendEmotionRPhi( Math.sqrt(Math.pow(' + value_valence + ', 2), Math.pow(' + value_arousal + ', 2) ), Math.atan2(' + value_arousal + ', ' + value_valence + ') * 180.0 / Math.PI' + ', -1);\n';
  var code = '';
  code += 'var r, phi, poly_index;\n';
  code += 'r = Math.sqrt(Math.pow(' + value_valence + ', 2), Math.pow(' + value_arousal + ', 2) );\n';
  code += 'phi = Math.atan2(' + value_arousal + ', ' + value_valence + ') * 180.0 / Math.PI;\n';
  code += 'poly_index = Math.floor(phi/18);\n';
  code += 'sendCommand(\'{"action": "set_expression", "poly_index": poly_index, "intensity": r, "time": -1}\');\n';
  code += '';
  return code;
};


Blockly.Blocks['expression_setrphi'] = {
  init: function() {
    this.appendDummyInput()
        .appendField(new Blockly.FieldImage("/static/libraries/font-awesome/svg/white/smile-o.svg", 16, 18, ""))
        .appendField("Set emotion to");
    this.appendValueInput("R")
        .setCheck("Number")
        .setAlign(Blockly.ALIGN_RIGHT)
        .appendField("R");
    this.appendValueInput("PHI")
        .setCheck("Number")
        .setAlign(Blockly.ALIGN_RIGHT)
        .appendField("Phi");
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.setColour(105);
    this.setTooltip('Set the current facial expression using Phi and R. Phi is a value in degrees, R ranges from 0.0 to 1.0.');
  }
};
Blockly.Lua['expression_setrphi'] = function(block) {
  var value_phi = Blockly.Lua.valueToCode(block, 'PHI', Blockly.Lua.ORDER_ATOMIC);
  var value_r = Blockly.Lua.valueToCode(block, 'R', Blockly.Lua.ORDER_ATOMIC);
  var code = 'Expression:set_emotion_r_phi(' + value_phi + ', ' + value_r +', true)\n';
  return code;
};
Blockly.JavaScript['expression_setrphi'] = function(block) {
  var value_phi = Blockly.JavaScript.valueToCode(block, 'PHI', Blockly.JavaScript.ORDER_ATOMIC);
  var value_r = Blockly.JavaScript.valueToCode(block, 'R', Blockly.JavaScript.ORDER_ATOMIC);
  // var code = 'robotSendEmotionRPhi(' + value_r + ', ' + value_phi +', -1);\n';

  var code = '';
  code += 'var r, phi, poly_index;\n';
  code += 'r = ' + value_r + ';\n';
  code += 'phi = ' + value_phi + ';\n';
  code += 'poly_index = Math.floor(phi/18);\n';
  code += 'sendCommand(\'{"action": "set_expression", "poly_index": poly_index, "intensity": r, "time": -1}\');\n';
  return code;
};
