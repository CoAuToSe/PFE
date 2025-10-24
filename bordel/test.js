const rclnodejs = require('rclnodejs');
/*
(async () => {
  await rclnodejs.init();
  const node = new rclnodejs.Node('client_test');
  const client = node.createClient('example_interfaces/srv/AddTwoInts', '/add_two_ints');
  console.log('Client created ✅');
})();*/
// test_rcl.js

process.env.RMW_IMPLEMENTATION = 'rmw_cyclonedds_cpp';

rclnodejs.init().then(() => {
  console.log('RCLNodeJS initialisé !');
  rclnodejs.shutdown();
}).catch(console.error);
