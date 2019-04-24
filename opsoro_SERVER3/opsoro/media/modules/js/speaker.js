var Speaker = function(config) {
    var self = this;
    self.base = Module;
    self.base(config);

    return self;
};
Speaker.prototype = new Module;

modules_definition['speaker'] = Speaker;
