const fs = require('fs');
const { Engine } = require('bpmn-engine');

// const source = fs.readFileSync('./process.bpmn', 'utf8');
const source = fs.readFileSync('./process_cond.bpmn', 'utf8');

const engine = new Engine({
  name: 'hello-world',
  source
});

const listener = {
  // petits logs pour voir la vie du process
  on: (eventName, cb) => {}, // placeholder requis par l'API interne
};

// Astuce : on peut aussi écouter via engine.once / engine.on
engine.on('activity.start', (api) => {
  console.log(`→ start: ${api.id} (${api.type})`);
});
engine.on('activity.end', (api) => {
  console.log(`✓ end:   ${api.id} (${api.type})`);
});
engine.on('end', () => {
  console.log('✔ process completed');
});

engine.execute({
  // variables disponibles dans les expressions/ScriptTasks si besoin
  variables: { cond: false },
  // on expose le console natif aux scripts BPMN
  services: { console }
});
