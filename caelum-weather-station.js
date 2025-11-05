
import * as m from 'zigbee-herdsman-converters/lib/modernExtend';

export default {
    zigbeeModel: ['caelum'],
    model: 'caelum',
    vendor: 'ESPRESSIF',
    description: 'Automatically generated definition',
    extend: [m.deviceEndpoints({"endpoints":{"1":1,"2":2,"3":3}}),
                                m.temperature(),
                                m.humidity(),
                                m.pressure(),
                                m.numeric({"name":"analog_input_2",
                                                "cluster":"genAnalogInput",
                                                "attribute":"presentValue",
                                                "reporting":{"min":"MIN","max":"MAX","change":1},
                                                "description":"analog_input_2",
                                                "access":"STATE_GET","endpointNames":["2"]}),
                                m.numeric({"name":"analog_input_3","cluster":"genAnalogInput","attribute":"presentValue","reporting":{"min":"MIN","max":"MAX","change":1},"description":"ana>
    ota: true,
};
