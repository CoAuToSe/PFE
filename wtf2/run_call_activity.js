const fs = require('fs');
const { Engine } = require('bpmn-engine');
const { EventEmitter } = require('events');

// const source = fs.readFileSync('./call_with_subprocess.bpmn', 'utf8');
const mainXml = fs.readFileSync('./say_call.bpmn', 'utf8');          // le parent avec <callActivity calledElement="say_something">
const subXml  = fs.readFileSync('./tello.bpmn', 'utf8'); 

// Un petit listener pour loguer clairement ce qui se passe
const listener = new EventEmitter();

listener.on('activity.start', (api) => {
  console.log(`▶️  start   ${api.id}  (${api.type})  in ${api.owner.name}`);
});

listener.on('activity.end', (api) => {
  console.log(`✅ end     ${api.id}  (${api.type})  in ${api.owner.name}`);
});

listener.on('activity.throw', (api, { message }) => {
  console.log(`⚠️ throw   ${api.id}  msg=${message}`);
});

listener.on('activity.error', (api, err) => {
  console.error(`❌ error   ${api.id}:`, err && err.message ? err.message : err);
});

// Instanciation de l'engine avec la définition qui contient parent + sous-processus
const engine = new Engine({
  name: 'call-demo',
  source: [mainXml, subXml],  
});

(async () => {
  console.log('--- Executing main process ---');
  const execution = await engine.execute({
    listener,
    // Vous pouvez passer des variables si besoin :
    // variables: { foo: 42 }
  });

  await execution.waitFor('end');

  console.log('--- Done ---');
})().catch((err) => {
  console.error('Engine execution failed:', err);
  process.exit(1);
});
