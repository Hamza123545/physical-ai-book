/**
 * Pyodide Preloader Plugin
 *
 * This plugin ensures Pyodide is loaded and ready before interactive Python components are used.
 * Note: The actual Pyodide CDN script is loaded via docusaurus.config.ts headTags.
 * This plugin just provides a convenient preloading interface if needed.
 */

module.exports = function (context, options) {
  return {
    name: 'docusaurus-plugin-pyodide-loader',

    // Inject client module for runtime availability checking
    getClientModules() {
      return [require.resolve('./pyodidePreloader.js')];
    },
  };
};
