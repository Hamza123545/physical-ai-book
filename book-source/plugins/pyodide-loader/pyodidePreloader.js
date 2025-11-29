/**
 * Pyodide Preloader Client Module
 *
 * Ensures Pyodide is available globally and provides a loading promise.
 */

let pyodideReadyPromise = null;

if (typeof window !== 'undefined') {
  pyodideReadyPromise = new Promise((resolve, reject) => {
    // Check if Pyodide is already loaded
    if (window.loadPyodide) {
      resolve(true);
      return;
    }

    // Wait for script to load
    const checkPyodide = setInterval(() => {
      if (window.loadPyodide) {
        clearInterval(checkPyodide);
        resolve(true);
      }
    }, 100);

    // Timeout after 30 seconds
    setTimeout(() => {
      clearInterval(checkPyodide);
      if (!window.loadPyodide) {
        reject(new Error('Pyodide failed to load within 30 seconds'));
      }
    }, 30000);
  });
}

export function waitForPyodide() {
  return pyodideReadyPromise;
}
