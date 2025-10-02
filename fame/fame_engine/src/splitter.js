'use strict';
const rclnodejs = require('rclnodejs');
const fs = require('fs');


var source = '';
var process_path = '../fame_engine/process/';
var process_dict = {};

fs.readFile(process_path + 'simple_scenario.bpmn', 'utf8', (err, data) => {
    if (err) {
        console.error(err)
        return
    }
    source = data;
})

function splitter() {
    // conversion from xml to object
    var convert = require('xml-js');
    var xml = source;
    var conversion = convert.xml2json(xml, { compact: true, spaces: 4 });
    conversion = conversion.replace(/'/g, '"');
    var conversion_obj = JSON.parse(conversion);
    var definitions = conversion_obj['bpmn:definitions'];
    var collab_objs = definitions['bpmn:collaboration']['bpmn:participant'];
    var process_objs = definitions['bpmn:process'];
    for (let i = 0; i < collab_objs.length; i++) {
        var process = collab_objs[i]._attributes;
        process_dict[process.processRef] = process.name;
    }

    var index = 0;
    Object.keys(process_dict).forEach(element => {
        var p = process_objs.find(proc => proc._attributes.id == element);
        var c = collab_objs.find(coll => coll._attributes.processRef == element);
        var temp = JSON.parse(JSON.stringify(definitions));
        temp['bpmn:collaboration']['bpmn:participant'] = temp['bpmn:collaboration']['bpmn:participant'][index];
        temp['bpmn:process'] = temp['bpmn:process'][index];
        var xml = convert.json2xml(temp, { compact: true, ignoreComment: true, spaces: 4 })
        var path = process_path + '/' + process_dict[element] + '.bpmn';
        process_dict[element] = {0: process_dict[element], 1: JSON.stringify(temp)}
    /**    fs.writeFile(path, xml, (err) => {
            if (err)
              console.log(err);
            else {
              console.log("File written successfully");
            }
          }); */
          index++;
    });
}


rclnodejs.init().then(() => {
    splitter();
    const node = rclnodejs.createNode('splitter_node');
    Object.keys(process_dict).forEach(element => {
        var namespace = process_dict[element][0]
        //console.log(namespace)
        const publisher = node.createPublisher('std_msgs/msg/String', namespace + '/bpmn_process');
        console.log(`Publishing process of ${namespace}`);
        var process = process_dict[element][1];
        publisher.publish(process);
    });





    rclnodejs.spin(node);
});