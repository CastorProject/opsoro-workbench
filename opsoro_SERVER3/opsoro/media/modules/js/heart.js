var Heart = function(config) {
    var self = this;
    self.base = Module;
    self.base(config);

    return self;
};
Heart.prototype = new Module;

modules_definition['heart'] = Heart;
