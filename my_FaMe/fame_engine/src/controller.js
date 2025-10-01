'use strict';

const { Engine } = require('bpmn-engine');
const bent = require('bent');
const rclnodejs = require('rclnodejs');
const { Parameter, ParameterDescriptor, ParameterType } = rclnodejs;
const { EventEmitter } = require('events');
const listener = new EventEmitter();
const fs = require('fs');
var convert = require('xml-js');

var topic_dict = {};

var source = '';
var engine_env = {};
var process_path = '../fame_engine/process/';
var process_dict = {};

//console.log(__dirname.split('/install')[0] + '/process')

function stripNS(obj) {
    if (Array.isArray(obj)) {
        return obj.map(stripNS);
    } else if (typeof obj === 'object' && obj !== null) {
        const newObj = {};
        for (const [key, value] of Object.entries(obj)) {
            const cleanKey = key.includes(':') ? key.split(':')[1] : key;
            newObj[cleanKey] = stripNS(value);
            //console.log("striping " + cleanKey);
        }
        return newObj;
    }
    return obj;
}

// function to load the bpmn file correctly
function writeProcess(source_process) {
    var conversion = convert.xml2json(source_process, { compact: true, spaces: 4 });
    conversion = conversion.replace(/'/g, '"');
    var conversion_obj_raw = JSON.parse(conversion);
    var conversion_obj = stripNS(conversion_obj_raw);
    //console.log(conversion_obj)
    // get inside the description of the process
    var processObj = conversion_obj['definitions']['process'];
    // get any callActivity
    var caObjs = processObj['callActivity'];
    // if there is a call activity inside the BPMN
    if (caObjs) {
        if (caObjs.length) {
            // for each item in call activity 
            for (let i = 0; i < caObjs.length; i++) {
                // get the name of the called activity
                var called_act = caObjs[i]._attributes.calledElement;
                // found the bpmn at the process location
                var ca_file = process_path + called_act + '.bpmn';
                // try to open the file
                var ca_source = fs.readFileSync(ca_file, 'utf8');
                // add it to the process dict
                process_dict[called_act] = [ca_source, false];
            }
        } else {
            // same as in the for loop (badly coded)
            var called_act = caObjs._attributes.calledElement;
            var ca_file = process_path + called_act + '.bpmn';
            var ca_source = fs.readFileSync(ca_file, 'utf8');
            process_dict[called_act] = [ca_source, false];
        }
    }
    // call the writeProcess recursively each new process that should be read
    Object.keys(process_dict).forEach(element => {
        if (!process_dict[element][1]) {
            process_dict[element][1] = true;
            writeProcess(process_dict[element][0]);
        }
    });
}
// try something with the subprocess (can't really tell as I don't have an example)
function merge_callActivity() {
    // conversion from xml to object
    var xml = source;
    var conversion = convert.xml2json(xml, { compact: true, spaces: 4 });
    conversion = conversion.replace(/'/g, '"');
    var conversion_obj_raw = JSON.parse(conversion);
    var conversion_obj = stripNS(conversion_obj_raw);
    // console.log(conversion_obj);
    // get inside the description of the process
    var processObj = conversion_obj['definitions']['process'];
    var caObjs = processObj['callActivity'];
    var spObjs = processObj['subProcess'];
    if (spObjs) { //the param triggeredByEvent = true blocks the execution of the subprocess
        if (spObjs.length) {
            for (let i = 0; i < caObjs.length; i++) {
                spObjs[i]._attributes.triggeredByEvent = false;
            }
        } else {
            spObjs._attributes.triggeredByEvent = false;
        }
        conversion_obj['definitions']['process']['subProcess'] = spObjs;
        //console.log(spObjs);
    }
    writeProcess(source);
    var arr_temp_process = [processObj];

    Object.keys(process_dict).forEach(element => {
        var conv = convert.xml2json(process_dict[element][0], { compact: true, spaces: 4 });
        conv = conv.replace(/'/g, '"');
        var ca_c_raw = JSON.parse(conv);
        var ca_c = stripNS(ca_c_raw);
        var process_push = ca_c['definitions']['process'];
        arr_temp_process.push(process_push);
        conversion_obj['definitions']['process'] = arr_temp_process;
    });

    var result = convert.json2xml(conversion_obj, { compact: true, ignoreComment: true, spaces: 4 });
    source = result;
}

// main execution
rclnodejs.init().then(() => {
    //start ROS node
    const node = rclnodejs.createNode('engine_node');

    // # Options for cmd 
    function getArg(flag, def = undefined) {
        const i = process.argv.indexOf(flag);
        return i >= 0 && i + 1 < process.argv.length ? process.argv[i + 1] : def;
    }
    const bpmn_name = getArg('--bpmn', node.namespace().replace('/', ''));
    // const userName = getArg('--name', 'world');
    // const mode     = getArg('--mode', 'normal');

    // // Déclarations + valeurs par défaut
    // node.declareParameter('bpmn_name', node.namespace().replace('/', '_'));
    // // node.declareParameter('frame_id', 'map');
    // // node.declareParameter('enabled', true);

    // // Lecture
    // const bpmn_name = node.getParameter('bpmn_name').value;
    // // const frameId = node.getParameter('frame_id').value;
    // // const enabled = node.getParameter('enabled').value;

    // node.declareParameters([
    //         new Parameter('bpmn_name', ParameterType.PARAMETER_STRING, node.namespace().replace('/', '')),
    //         // new Parameter('rate', ParameterType.PARAMETER_INTEGER, 10),
    //         // new Parameter('frame_id', ParameterType.PARAMETER_STRING, 'map'),
    //     ], [
    //         new ParameterDescriptor('bpmn_name', ParameterType.PARAMETER_STRING, ''),
    //         // new ParameterDescriptor('rate', ParameterType.PARAMETER_INTEGER, 'Hz'),
    //         // new ParameterDescriptor('frame_id', ParameterType.PARAMETER_STRING, 'TF frame'),
    // ]);
    // const bpmn_name = node.getParameter('bpmn_name').value;

    // var process_name = node.namespace().replace('/', '');
    // source = (fs.readFileSync(process_path + process_name + '.bpmn', 'utf8'));
    source = (fs.readFileSync(process_path + bpmn_name + '.bpmn', 'utf8'));
    merge_callActivity();
    //fs.writeFileSync(process_path+'res.bpmn', source);

    var tstart = 0;
    var tfinish = 0;

    // initialization of the engine
    const engine = Engine({
        name: 'fame',
        source
    });
    /*
        listener.on('flow.take', (flow) => {
            console.log(`flow.take <${flow.id}> was taken`);
        });
    */
    listener.on('activity.start', (activity) => {
        if (tstart == 0) tstart = activity.messageProperties.timestamp;
        handleDataObj(activity);
        // console.log(`activity.start <${activity.id}> was taken`);
    });

    listener.on('activity.end', (activity) => {
        tfinish = activity.messageProperties.timestamp;
        // add activity variables to global ones
        addVars(activity.environment.variables);
        engine_env = engine.environment;
        //console.log('HERE:', engine_env.variables);
        // console.log(`activity.end <${activity.id}> was released`);
    });

    /* 
    listener.on('activity.wait', (wait) => {
        console.log(`wait <${wait.id}> was taken`);
    });

    listener.on('activity.throw', (throwev) => {
        console.log(`throw <${throwev.id}> was taken`);
    });

    listener.on('activity.error', (errorev) => {
        console.log(`error <${errorev.id}> was taken`);
    });
    */

    function addVars(var_activity) {
        Object.keys(var_activity).forEach(element => {
            if (element != 'ros_node' && element != 'fields' && element != 'content' && element != 'properties') {
                if (!(element in Object.keys(engine.environment.variables))) {
                    var vs = new Object();
                    vs[element] = var_activity[element];
                    engine.environment.assignVariables(vs)
                }
            }
        });
    }

    /**
     * Management of signal throwing // sus, seems to not work, to be checked
     */
    engine.broker.subscribeTmp('event', 'activity.signal', (routingKey, msg) => { // routingKey = activity.signal
        let topic_name = msg.content.name
        let message_type;
        let message_payload;
        let check = false;
        //console.log(msg);
        const regexpr = /\${(.*?)\}/g; // all variables are identified through ${...}

        for (let key in topic_dict) {
            if (key === topic_name) {
                message_type = topic_dict[key][0];
                message_payload = topic_dict[key][1];
                var tempvar = message_payload.match(regexpr);
                // check if there are variables that needs a value assignment
                if (tempvar) {
                    for (let i = 0; i < tempvar.length; i++) {
                        var val = tempvar[i];
                        if (val.startsWith('$')) {
                            var variable = val.substring(2, val.length - 1); // removes ${}
                            //console.log(engine_env.variables);
                            var value = engine_env.variables[variable];
                            //console.log(value);
                            message_payload = message_payload.replace(val, value); // replace variable with value
                            topic_dict[key][1] = message_payload; // update topic dictionary
                        }
                    }
                }
                check = true;
                break;
            }
        }
        // Publish ros topic
        if (check) {
            engine.execution.signal(msg.content.message, { ignoreSameDefinition: true });
            console.log(`Publishing message on ${topic_name}: ` + message_payload);
            const publisher = node.createPublisher(message_type, '/' + topic_name);
            var message_obj_raw = JSON.parse(message_payload); // conversion from string to obj
            var message_obj = stripNS(message_obj_raw);
            publisher.publish(message_obj);
        }
    }, { noAck: true });


    engine.execute({
        listener,
        variables: {
            ros_node: node
        },
        services: {
            get: bent('json'),
            set,

            // to be able to print in the console
            console,

            // timers Node déjà exposés si besoin
            setTimeout,
            clearTimeout,
            setInterval,
            clearInterval,

        },
        moddleOptions: {
            camunda: require('camunda-bpmn-moddle/resources/camunda'),
        },
        extensions: {
            camunda: camundaExtProperties,
        }
    });

    engine.on('end', (execution) => {
        // console.log('Ended:', process_name);
        console.log('Ended:', bpmn_name);
    });

    // ????
    function set(activity, name, value) {
        activity.logger.debug('set', name, 'to', value);
    }

    /**
     * Manages camunda external properties
     * @param {*} activity 
     */
    function camundaExtProperties(activity) {
        // if there is no activity.behaviour.extensionElements: pass (why .behaviour ???)
        if (!activity.behaviour.extensionElements) return;
        let msg_type; // message type
        let ref_topic; // topic name
        let msg_payload; // massage payload
        for (const extn of activity.behaviour.extensionElements.values) {
            // only do the following operation on the 'properties' block
            if (extn.$type === 'properties') {
                ref_topic = activity.name;
                let prop = extn.$children; // properties data
                msg_type = prop[0].value; // TO FIX -> non ha controlli
                // if it is a throw signal
                if (prop.length > 1) {
                    msg_payload = prop[1].value;
                    topic_dict[ref_topic] = [msg_type, msg_payload]                 // save properties parameters in the topic dictionary
                } else { // it is a catch
                    console.log('Subscribed to: ', ref_topic);
                    // added '/' to avoid remap of topics
                    node.createSubscription(msg_type, '/' + ref_topic, (msg) => { //create ROS subscription
                        console.log(`Received message: `, msg);
                        //activity.environment.assignVariables();
                        Object.keys(msg).forEach(element => {
                            // assing to global varibles the payload of the signal
                            const find = Object.keys(activity.environment.variables).find(v => v.startsWith(element));
                            if (find) { // check if there is a matching variable
                                var value = msg[element];
                                activity.environment.variables[find] = value;
                            }
                        });
                        activity.getApi().sendApiMessage('signal'); // forces signal catching
                    });
                }
            }
        }
    }

    /**
     * Assigns data object values to global variables
     * @param {*} activity 
     */
    function handleDataObj(activity) {
        if (!activity.owner.behaviour.dataInputAssociations) {
            if (!activity.owner.behaviour.dataOutputAssociations) return; // check if there are associated data objects
        }
        // conversion from xml to object
        var xml = source;
        var conversion = convert.xml2json(xml, { compact: true, spaces: 4 });
        conversion = conversion.replace(/'/g, '"');
        var conversion_obj_raw = JSON.parse(conversion);
        var conversion_obj = stripNS(conversion_obj_raw);
        // data objects extraction
        var processObjs = conversion_obj['definitions']['process'];
        var dataObjs = [];
        if (processObjs.length) {
            for (let i in processObjs) {
                var dObj = processObjs[i]['dataObjectReference'];
                if (dObj) {
                    if (dObj.length) {
                        for (let j in dObj) {
                            dataObjs.push(dObj[j]);
                        }
                    } else {
                        dataObjs.push(dObj);
                    }
                }
            }
        }

        // activities data objects extraction
        if (!activity.owner.behaviour.dataInputAssociations) {
            var act_obj = activity.owner.behaviour.dataOutputAssociations[0].behaviour.targetRef.id;
        } else
            var act_obj = activity.owner.behaviour.dataInputAssociations[0].behaviour.sourceRef.id;
        for (let da in dataObjs) {
            var obj = dataObjs[da]
            var obj_id = obj._attributes.id;
            if (act_obj === obj_id) {
                var variable = new Object();
                // if there are values assigned to the variable
                if (obj['extensionElements']) {
                    var properties = obj['extensionElements']['properties']['property'];
                    //console.log(properties);
                    if (properties.length > 1) {
                        for (let p_index in properties) {
                            var p = properties[p_index]._attributes;
                            // if (!p.value.startsWith('$'))
                            variable[p.name] = p.value;
                        }
                    } else {
                        var p = properties._attributes;
                        //if (!p.value.startsWith('$'))
                        variable[p.name] = p.value;
                    }

                    // add data object variables to the global environment
                    activity.environment.assignVariables(variable);

                    Object.keys(engine.environment.variables).forEach(element => {
                        if (!(element in Object.keys(activity.environment.variables))) {
                            var v = new Object();
                            v[element] = (engine.environment.variables[element]);
                            // console.log(v);
                            activity.environment.assignVariables(v);
                        }
                    });
                    engine.environment.assignVariables(variable);
                }
            }
        }
    }

    rclnodejs.spin(node);
});

