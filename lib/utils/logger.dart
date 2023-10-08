import 'package:logger/logger.dart';
//export 'package:cli_util/cli_logging.dart' show Progress;

NRFLogger? logger;

/// Flutter Launcher Icons Logger
class NRFLogger {
  late Logger _logger;

  /// Returns true if this is a verbose logger
  final bool isVerbose;

  /// Gives access to internal logger
  Logger get rawLogger => _logger;

  /// Creates a instance of [FLILogger].
  /// In case [isVerbose] is `true`,
  /// it logs all the [verbose] logs to console
  NRFLogger(this.isVerbose) {
    _logger = Logger(
      printer: PrettyPrinter(
          methodCount: 2, // Number of method calls to be displayed
          errorMethodCount: 8, // Number of method calls if stacktrace is provided
          lineLength: 120, // Width of the output
          colors: true, // Colorful log messages
          printEmojis: true, // Print an emoji for each log message
          printTime: true // Should each log print contain a timestamp
      ),
      level: Level.all
    );
  }

  /// Logs error messages
  void error(Object? message) => _logger.e('Error Log', error: '⚠️ $message');

  /// Prints to console if [isVerbose] is true
  void verbose(Object? message){
    if(isVerbose){
      _logger.t(message.toString());
    }
  }
  /// Prints to console if [isVerbose] is true
  void warning(Object? message) => _logger.w(message.toString());
  /// Prints to console
  void info(Object? message) => _logger.i(message.toString());
}