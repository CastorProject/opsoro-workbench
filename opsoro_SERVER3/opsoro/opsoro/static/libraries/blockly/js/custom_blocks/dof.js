Blockly.Blocks['dof_set'] = {
  init: function() {
    this.appendDummyInput()
        .appendField(new Blockly.FieldImage("/static/libraries/font-awesome/svg/white/gears.svg", 16, 18, ""))
        .appendField("Set dof ")
        .appendField(new Blockly.FieldDropdown(model.blockly_dofs), "DOF");
    this.appendValueInput("VALUE")
        .setCheck("Number")
        .appendField("to value");
    this.setInputsInline(true);
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.setColour(205);
    this.setTooltip('Sets the value of one DOF. Value should be between -1.0 and 1.0.');
  }
};
Blockly.Lua['dof_set'] = function(block) {
  var dropdown_tags = block.getFieldValue('DOF');
  var value_pos = Blockly.Lua.valueToCode(block, 'VALUE', Blockly.Lua.ORDER_ATOMIC);
  var code = 'Robot:set_dof("' + dropdown_tags + '", ' + value_pos + ')\n';
  return code;
};
Blockly.JavaScript['dof_set'] = function(block) {
  var dropdown_tags = block.getFieldValue('DOF');
  var value_pos = Blockly.JavaScript.valueToCode(block, 'VALUE', Blockly.JavaScript.ORDER_ATOMIC);
  // var code = 'robotSendDOF([{"name": "' + dropdown_tags + '", "value": ' + value_pos + '}])\n';
  var code = 'sendCommand(\'{"action": "set_dofs", "dofs": [{"name": "' + dropdown_tags + '", "value": ' + value_pos + '}]}\');\n';
  return code;
};
