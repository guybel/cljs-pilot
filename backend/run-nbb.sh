#!/bin/bash
cd /home/guy/Documents/pilot/cljs-pilot/backend
export NODE_OPTIONS="--input-type=module"
node -e "
import('./node_modules/nbb/lib/nbb_main.js').then(async (nbb) => {
  process.argv = ['node', 'nbb', '-cp', 'src', 'main.cljs'];
  try {
    if (nbb.default && typeof nbb.default === 'function') {
      await nbb.default();
    } else if (typeof nbb === 'function') {
      await nbb();
    } else {
      console.log('nbb module loaded');
    }
  } catch (e) {
    console.error('Error:', e.message);
  }
}).catch(console.error);
"
