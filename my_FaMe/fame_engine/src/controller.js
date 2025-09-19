'use strict';

const { Engine } = require('bpmn-engine');
const bent = require('bent');
const rclnodejs = require('rclnodejs');
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

function writeProcess(source_process) {
  var conversion = convert.xml2json(source_process, { compact: true, spaces: 4 });
  conversion = conversion.replace(/'/g, '"');
  var conversion_obj = JSON.parse(conversion);
  var processObj = conversion_obj['bpmn:definitions']['bpmn:process'];
  var caObjs = processObj['bpmn:callActivity'];
  if (caObjs) {
    if (caObjs.length) {
      for (let i = 0; i < caObjs.length; i++) {
        var called_act = caObjs[i]._attributes.calledElement;
        var ca_file = process_path + called_act + '.bpmn';
        var ca_source = fs.readFileSync(ca_file, 'utf8');
        process_dict[called_act] = [ca_source, false];
      }
    } else {
      var called_act = caObjs._attributes.calledElement;
      var ca_file = process_path + called_act + '.bpmn';
      var ca_source = fs.readFileSync(ca_file, 'utf8');
      process_dict[called_act] = [ca_source, false];
    }
  }
  Object.keys(process_dict).forEach(element => {
    if (!process_dict[element][1]) {
      process_dict[element][1] = true;
      writeProcess(process_dict[element][0]);
    }
  });
}

function mergeCallActivity() {
  // conversion from xml to object
  var xml = source;
  var conversion = convert.xml2json(xml, { compact: true, spaces: 4 });
  conversion = conversion.replace(/'/g, '"');
  var conversion_obj = JSON.parse(conversion);
  // console.log(conversion_obj);
  var processObj = conversion_obj['bpmn:definitions']['bpmn:process'];
   var caObjs = processObj['bpmn:callActivity'];
  var spObjs = processObj['bpmn:subProcess'];
  if (spObjs) { //the param triggeredByEvent = true blocks the execution of the subprocess
    if (spObjs.length) {
      for (let i = 0; i < caObjs.length; i++) {
        spObjs[i]._attributes.triggeredByEvent = false;
      }
    } else {
      spObjs._attributes.triggeredByEvent = false;
    }
    conversion_obj['bpmn:definitions']['bpmn:process']['bpmn:subProcess'] = spObjs;
    //console.log(spObjs);
  }
  writeProcess(source);
  var arr_temp_process = [processObj];

  Object.keys(process_dict).forEach(element => {
    var conv = convert.xml2json(process_dict[element][0], { compact: true, spaces: 4 });
    conv = conv.replace(/'/g, '"');
    var ca_c = JSON.parse(conv);
    var process_push = ca_c['bpmn:definitions']['bpmn:process'];
    arr_temp_process.push(process_push);
    conversion_obj['bpmn:definitions']['bpmn:process'] = arr_temp_process;
  });

  var result = convert.json2xml(conversion_obj, { compact: true, ignoreComment: true, spaces: 4 });
  source = result;
}

rclnodejs.init().then(() => {
  //start ROS node
  const node = rclnodejs.createNode('engine_node');
  var process_name = node.namespace().replace('/', '');
  source = (fs.readFileSync(process_path + process_name + '.bpmn', 'utf8'));
  mergeCallActivity();
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
    addVars(activity.environment.variables); // add activity variables to global ones
    engine_env = engine.environment;
    //console.log('HERE:', engine_env.variables);
    // console.log(`activity.end <${activity.id}> was released`);
  });

  /*  listener.on('activity.wait', (wait) => {
      console.log(`wait <${wait.id}> was taken`);
    });
  
    listener.on('activity.throw', (throwev) => {
      console.log(`throw <${throwev.id}> was taken`);
    });
  
    listener.on('activity.error', (errorev) => {
      console.log(`error <${errorev.id}> was taken`);
    });*/

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
   * Management of signal throwing
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
      var message_obj = JSON.parse(message_payload); // conversion from string to obj
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
      console,
    },
    moddleOptions: {
      camunda: require('camunda-bpmn-moddle/resources/camunda'),
    },
    extensions: {
      camunda: camundaExtProperties,
    }
  });

  engine.on('end', (execution) => {
    console.log('Ended:', process_name);
  });

  function set(activity, name, value) {
    activity.logger.debug('set', name, 'to', value);
  }

  /**
   * Manages camunda external properties
   * @param {*} activity 
   */
  function camundaExtProperties(activity) {
    if (!activity.behaviour.extensionElements) return;
    let msg_type; // message type
    let ref_topic; // topic name
    let msg_payload; // massage payload
    for (const extn of activity.behaviour.extensionElements.values) {
      if (extn.$type === 'camunda:properties') {
        ref_topic = activity.name;
        let prop = extn.$children; // properties data
        msg_type = prop[0].value; // TO FIX -> non ha controlli
        // if it is a throw signal
        if (prop.length > 1) {
          msg_payload = prop[1].value;
          topic_dict[ref_topic] = [msg_type, msg_payload]         // save properties parameters in the topic dictionary
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
    var conversion_obj = JSON.parse(conversion);
    // data objects extraction
    var processObjs = conversion_obj['bpmn:definitions']['bpmn:process'];
    var dataObjs = [];
    if (processObjs.length) {
      for (let i in processObjs) {
        var dObj = processObjs[i]['bpmn:dataObjectReference'];
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
        if (obj['bpmn:extensionElements']) {
          var properties = obj['bpmn:extensionElements']['camunda:properties']['camunda:property'];
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

