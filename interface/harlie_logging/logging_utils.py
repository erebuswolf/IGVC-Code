import logging
import logging.handlers
import settings

handler = logging.handlers.RotatingFileHandler(settings.logger['filename'], maxBytes=settings.logger['max_filesize'], backupCount=settings.logger['numBackups'])
formatter = logging.Formatter('"%(levelname)s","%(name)s","%(asctime)s","%(message)s"')
handler.setFormatter(formatter)