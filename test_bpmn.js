const { Engine } = require('bpmn-engine');
const fs = require('fs');

const source = fs.readFileSync('/home/dell/PFE/my_FaMe/fame_engine/process/hello_world.bpmn', 'utf8');  // adapte le chemin si besoin

const engine = new Engine({
  name: 'hello-world-test',
  source
});

engine.execute({
  services: {
    console
  }
}, (err, execution) => {
  if (err) {
    console.error('❌ Erreur d’exécution :', err);
    return;
  }

  console.log('✅ Le processus a démarré avec succès !');

  execution.once('end', () => {
    console.log('🏁 Processus terminé.');
  });
});
