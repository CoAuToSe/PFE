const fs = require('fs');
const path = require('path');
const { Engine } = require('bpmn-engine');
const { EventEmitter } = require('events');

/**
 * Extrait le contenu interne d'une balise <bpmn:definitions>…</bpmn:definitions>
 */
function extractInnerDefinitions(xml, file) {
  const m = xml.match(/<bpmn:definitions[\s\S]*?>([\s\S]*?)<\/bpmn:definitions>/);
  if (!m) throw new Error(`Pas de <bpmn:definitions> trouvé dans: ${file}`);
  return m[1];
}

/**
 * Détecte si au moins un document utilise un namespace donné (ex: "camunda:")
 */
function anyUsesNamespace(xmls, nsPrefixWithColon) {
  const re = new RegExp(`\\b${nsPrefixWithColon.replace(':', '\\:')}`);
  return xmls.some((x) => re.test(x));
}

/**
 * Construit une <bpmn:definitions> unique contenant les inner-defs de N fichiers.
 * Ajoute automatiquement les namespaces courants (camunda si détecté).
 */
function makeBundle(xmls) {
  const needsCamunda = anyUsesNamespace(xmls, 'camunda:');

  const header = [
    `<?xml version="1.0" encoding="UTF-8"?>`,
    `<bpmn:definitions xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"`,
    `  xmlns:bpmn="http://www.omg.org/spec/BPMN/20100524/MODEL"`,
    `  xmlns:bpmndi="http://www.omg.org/spec/BPMN/20100524/DI"`,
    `  xmlns:dc="http://www.omg.org/spec/DD/20100524/DC"`,
    `  xmlns:di="http://www.omg.org/spec/DD/20100524/DI"${needsCamunda ? `\n  xmlns:camunda="http://camunda.org/schema/1.0/bpmn"` : ''}`,
    `  targetNamespace="http://bpmn.io/schema/bpmn">`,
  ].join('\n');
  console.log(header)

  const body = xmls
    .map(({ xml, file }) => extractInnerDefinitions(xml, file))
    .join('\n');

  return `${header}\n${body}\n</bpmn:definitions>`;
}

/**
 * Simple logger d'exécution
 */
function makeListener() {
  const listener = new EventEmitter();
  listener.on('activity.start', (api) =>
    console.log(`▶️  start   ${api.id} (${api.type}) in ${api.owner.name}`)
  );
  listener.on('activity.end', (api) =>
    console.log(`✅ end     ${api.id} (${api.type}) in ${api.owner.name}`)
  );
  listener.on('activity.error', (api, err) =>
    console.error(`❌ error   ${api.id}:`, err?.message || err)
  );
  listener.on('error', (err) => console.error('Engine error:', err));
  return listener;
}

// --------- MAIN ---------
(async () => {
  // Passe tes fichiers BPMN en arguments: node bundle_and_run.js main.bpmn sub1.bpmn sub2.bpmn ...
  const files = process.argv.slice(2);
  if (!files.length) {
    console.error('Usage: node bundle_and_run.js main.bpmn sub1.bpmn [...]');
    process.exit(1);
  }

  // Lecture des fichiers
  const sources = files.map((f) => ({
    file: f,
    xml: fs.readFileSync(path.resolve(f), 'utf8'),
  }));

  // Construction du bundle
  const bundle = makeBundle(sources);

  // (Optionnel) écrire le bundle sur disque pour debug
  fs.writeFileSync('bundle.bpmn', bundle, 'utf8');
  console.log('📦 bundle généré -> bundle.bpmn');

  // Exécution
  const engine = new Engine({ name: 'bundle-exec', source: bundle });
  const execution = await engine.execute({ listener: makeListener(), services: { console }});

  await execution.waitFor('end');
  console.log('--- done ---');
})().catch((err) => {
  console.error(err);
  process.exit(1);
});
